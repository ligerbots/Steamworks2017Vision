package erik.android.vision.visiontest;

import android.util.Log;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class ImageProcessor implements Runnable {
    private static final String TAG = "ImageProcessor";

    private Thread mProcessingThread;

    private final Object mProcessingLock = new Object();
    private final Object mWaitLock = new Object();
    boolean enabled = false;
    private Mat mProcessingHsvMat;
    private MatOfPoint mProcessingLargestContour = null;
    private Point[] mProcessingPolyFit = null;

    private Mat mCalibGrayMat = null;

    private NTColorPicker mFilterColorRange;
    private FpsCounter mProcessingFps;
    private Calibration mCalibration;

    private MatOfPoint2f polyFit = new MatOfPoint2f();
    List<MatOfPoint> contours = new LinkedList<>();
    Mat hierarchy = new Mat();

    private NetworkTable result;

    public ImageProcessor(Calibration calibration) {
        mFilterColorRange = new NTColorPicker(Parameters.purpose.visionTable + "/colorRange", NTColorPicker.ColorMode.HSV);
        mProcessingFps = new FpsCounter("ProcessingThread");
        mCalibration = calibration;
        result = NetworkTable.getTable(Parameters.purpose.visionTable + "/result");

        mProcessingThread = new Thread(this);
        mProcessingThread.setName("OpenCV Image Processor");
        mProcessingThread.setDaemon(true);
        mProcessingThread.start();
    }

    public void postUpdateAndDraw(Mat sourceBgrMat, Mat drawingRgbMat) {
        synchronized (mProcessingLock) {
            if (mProcessingThread.getState() == Thread.State.WAITING) {
                if (mProcessingHsvMat == null) {
                    mProcessingHsvMat = new Mat();
                }
                Imgproc.cvtColor(sourceBgrMat, mProcessingHsvMat, Imgproc.COLOR_RGB2HSV);
                if (mCalibGrayMat == null) {
                    mCalibGrayMat = new Mat();
                }
                if (mCalibration.isEnabled()) {
                    Imgproc.cvtColor(sourceBgrMat, mCalibGrayMat, Imgproc.COLOR_BGR2GRAY);
                    //mCalibration.renderFrame(drawingRgbMat);
                }
                mProcessingLock.notify();
            }

            if (mProcessingLargestContour != null) {
                Imgproc.drawContours(drawingRgbMat,
                        Collections.singletonList(mProcessingLargestContour), 0,
                        new Scalar(255, 0, 0));
            }
            if (mProcessingPolyFit != null) {
                org.opencv.core.Point[] localCopy = mProcessingPolyFit;
                for (int i = 0; i < localCopy.length; ++i) {
                    Imgproc.line(drawingRgbMat, localCopy[i], localCopy[(i + 1) % localCopy.length],
                            new Scalar(0, 255, 0));
                    Imgproc.putText(drawingRgbMat, Integer.toString(i), localCopy[i],
                            Core.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 0, 255));
                }
            }
        }

        if (Parameters.purpose == Parameters.Purpose.BOILER) {
            Core.transpose(drawingRgbMat, drawingRgbMat);
            Core.flip(drawingRgbMat, drawingRgbMat, 1);
            Imgproc.line(drawingRgbMat, new Point(Parameters.CAPTURE_SIZE.getHeight() / 2, 0),
                    new Point(Parameters.CAPTURE_SIZE.getHeight() / 2,
                            Parameters.CAPTURE_SIZE.getWidth()), new Scalar(0, 255, 0));
        }
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        synchronized (mWaitLock) {
            mWaitLock.notifyAll();
        }
    }

    @Override
    public void run() {
        //noinspection InfiniteLoopStatement
        while (true) {
            try {
                Log.i(TAG, "Processing thread waiting");

                if (!enabled) {
                    synchronized (mWaitLock) {
                        mWaitLock.wait(5000);
                    }
                    if(!enabled) {
                        continue;
                    }
                }

                synchronized (mProcessingLock) {
                    try {
                        mProcessingLock.wait();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                Log.i(TAG, "Processing thread loop");
                long st = System.currentTimeMillis();
                // if calibration is enabled, do it
                if (mCalibration.isEnabled()) {
                    mCalibration.findPattern(mCalibGrayMat);
                }
                // send image histogram
                mFilterColorRange.setHistogram(mProcessingHsvMat);

                // Process gear lift
                boolean found;
                if (Parameters.purpose == Parameters.Purpose.BOILER) {
                    found = findBoilerTarget(mProcessingHsvMat);
                } else {
                    found = findGearTarget(mProcessingHsvMat);
                }

                if (found) {
                    Point[] polyFitPts = polyFit.toArray();
                    Communications.root.putString("Image_Coords", String.format("%s%n%s%n%s%n%s",
                            polyFitPts[0].toString(),
                            polyFitPts[1].toString(),
                            polyFitPts[2].toString(),
                            polyFitPts[3].toString()));

                    // calculate camera transform
                    MatOfPoint3f objPoints = new MatOfPoint3f();
                    double[] targetSize = Parameters.getTargetSize();

                    if (Double.isNaN(targetSize[0]) || Double.isNaN(targetSize[1])) {
                        Communications.root.putString("Status", "Waiting for robot data");
                    } else {
                        if (Parameters.purpose == Parameters.Purpose.BOILER) {
                            objPoints.fromArray(
                                    new Point3( targetSize[1] / 2,  targetSize[0] / 2, 0),
                                    new Point3(-targetSize[1] / 2,  targetSize[0] / 2, 0),
                                    new Point3(-targetSize[1] / 2, -targetSize[0] / 2, 0),
                                    new Point3( targetSize[1] / 2, -targetSize[0] / 2, 0)
                            );
                        } else {
                            objPoints.fromArray(
                                    new Point3(-targetSize[0] / 2, -targetSize[1] / 2, 0),
                                    new Point3(-targetSize[0] / 2, targetSize[1] / 2, 0),
                                    new Point3(targetSize[0] / 2, targetSize[1] / 2, 0),
                                    new Point3(targetSize[0] / 2, -targetSize[1] / 2, 0)
                            );
                        }

                        double[] cameraMatrixA = mCalibration.getCameraMatrixArray();
                        double[] distortionCoefficientsA =
                                mCalibration.getDistortionCoefficientsArray();

                        if (cameraMatrixA.length > 0 && distortionCoefficientsA.length > 0) {
                            Mat cameraMatrix = new Mat();
                            MatOfDouble distortCoeff = new MatOfDouble();
                            Mat.eye(3, 3, CvType.CV_64FC1).copyTo(cameraMatrix);
                            cameraMatrix.put(0, 0, 1.0);
                            Mat.zeros(5, 1, CvType.CV_64FC1).copyTo(distortCoeff);
                            Mat rvec = new Mat(), tvec = new Mat();

                            for (int i = 0; i < 3; i++) {
                                for (int j = 0; j < 3; j++) {
                                    cameraMatrix.put(i, j, cameraMatrixA[i * 3 + j]);
                                }
                            }
                            for (int i = 0; i < distortionCoefficientsA.length; i++) {
                                distortCoeff.put(i, 0, distortionCoefficientsA[i]);
                            }

                            Calib3d.solvePnP(objPoints, polyFit, cameraMatrix, distortCoeff,
                                    rvec, tvec);

                            Mat rmat = new Mat();

                            Calib3d.Rodrigues(rvec, rmat);
                            rvec.release();
                            rvec = new Mat(1, 3, CvType.CV_64F);
                            rvec.put(0, 0,
                                    Core.fastAtan2((float) rmat.get(1, 2)[0],
                                            (float) rmat.get(2, 2)[0]),
                                    Core.fastAtan2((float) -rmat.get(2, 0)[0],
                                            (float) Math.sqrt(rmat.get(1, 2)[0] * rmat.get(1, 2)[0]
                                                    + rmat.get(2, 2)[0] * rmat.get(2, 2)[0])),
                                    Core.fastAtan2((float) rmat.get(1, 0)[0],
                                            (float) rmat.get(0, 0)[0])
                            );

                            double[] rvecDouble = new double[(int) rvec.total()];
                            rvec.get(0, 0, rvecDouble);
                            double[] tvecDouble = new double[(int) tvec.total()];
                            tvec.get(0, 0, tvecDouble);

                            double imgCx = (polyFitPts[0].x + polyFitPts[1].x  + polyFitPts[2].x  + polyFitPts[3].x)/4;
                            double imgCy = (polyFitPts[0].y + polyFitPts[1].y  + polyFitPts[2].y  + polyFitPts[3].y)/4;

                            result.putNumber("rvec_pitch", rvecDouble[0]);
                            result.putNumber("rvec_yaw", rvecDouble[1]);
                            result.putNumber("rvec_roll", rvecDouble[2]);
                            result.putNumber("tvec_x", tvecDouble[0]);
                            result.putNumber("tvec_y", tvecDouble[1]);
                            result.putNumber("tvec_z", tvecDouble[2]);
                            result.putNumber("img_cx", imgCx);
                            result.putNumber("img_cy", imgCy);
                            Communications.dataServerSendData(rvecDouble[0], rvecDouble[1],
                                    rvecDouble[2], tvecDouble[0], tvecDouble[1], tvecDouble[2], imgCx, imgCy);
                            Communications.root.putString("Status", "OK");
                        }
                    }
                }

                Log.i(TAG, "Processing thread dt: " + (System.currentTimeMillis() - st));

                if (found) {
                    mProcessingPolyFit = polyFit.toArray();
                } else {
                    mProcessingPolyFit = null;
                }

                mProcessingFps.feed();
                Log.i(TAG, "Processing thread loop done");
            } catch (Throwable t) {
                Log.e(TAG, "Processing thread error!", t);
                Communications.root.putString("Status", "Crashed");
            }
        }

    }

    static class ContourInfo {
        MatOfPoint contour;
        double area;
        int numSourceContours;

        public ContourInfo(MatOfPoint contour, double area, int numSourceContours) {
            this.contour = contour;
            this.area = area;
            this.numSourceContours = numSourceContours;
        }
    }

    private void releaseList(List<? extends Mat> list) {
        for (Mat mat : list) {
            mat.release();
        }
        list.clear();
    }

    private void releaseList0(List<ContourInfo> list) {
        for (ContourInfo info : list) {
            info.contour.release();
        }
        list.clear();
    }

    private boolean findGearTarget(Mat src) {
        // filter out green
        Core.inRange(src, mFilterColorRange.getLower(),
                mFilterColorRange.getUpper(), src);

        // detect all contours
        Mat contourCopy = src.clone();
        Imgproc.findContours(contourCopy, contours, hierarchy, Imgproc.RETR_CCOMP,
                Imgproc.CHAIN_APPROX_SIMPLE);

        List<ContourInfo> combinedContours = new LinkedList<>();

        // combine contours that are close to each other. This links together the contours that may
        // be split up by the gear lift peg
        outer:
        while (contours.size() > 0) {
            MatOfPoint first = contours.remove(0);
            List<Point> combined = new ArrayList<>();
            combined.addAll(first.toList());
            int numSources = 1;

            for (int i = 0; i < contours.size(); i++) {
                MatOfPoint other = contours.get(i);
                if (contourMinDist(first, other) < 30 * 30) {
                    combined.addAll(other.toList());
                    contours.remove(i);
                    numSources++;
                    i--;
                }
            }

            // check if the entire contour is reasonably within bounds, otherwise we risk garbage
            // values from a clipped target
            for (Point point : combined) {
                double x = point.x;
                double y = point.y;
                if (x < 2 || x > src.width() - 2 || y < 2 || y > src.height() - 2) {
                    continue outer;
                }
            }

            // combine contours using convex hull
            MatOfInt hullIdx = new MatOfInt();
            MatOfPoint all = new MatOfPoint();
            all.fromList(combined);
            Imgproc.convexHull(all, hullIdx);
            MatOfPoint normalizedCombinedContour = normalizeHull(all, hullIdx);

            // check if the result is reasonably vertical
            Rect boundingRect = Imgproc.boundingRect(normalizedCombinedContour);
            if (((double) boundingRect.height) / boundingRect.width < 1.5) {
                continue;
            }

            hullIdx.release();
            all.release();
            double area = Imgproc.contourArea(normalizedCombinedContour);
            System.out.println("new combined contour " + numSources + "/" + area);
            combinedContours.add(new ContourInfo(normalizedCombinedContour, area, numSources));
        }

        // get the 2 largest contours in the image that passed the first operation
        Collections.sort(combinedContours, new Comparator<ContourInfo>() {
            @Override
            public int compare(ContourInfo o1, ContourInfo o2) {
                return (int) Math.signum(o2.area - o1.area);
            }
        });

        if (combinedContours.size() < 2) {
            Log.i(TAG, "No gear target found: not enough contours");
            Communications.root.putString("Status", "<2 contours in frame");
            releaseList0(combinedContours);
            contourCopy.release();
            return false;
        }

        // if there are 3 or more contours, detect the 2 closest to each other. These are likely
        // the gear target we want

        if (combinedContours.size() >= 3) {
            ContourInfo c0 = combinedContours.get(0);
            ContourInfo c1 = combinedContours.get(1);
            ContourInfo c2 = combinedContours.get(2);

            double a0 = c0.area;
            double a1 = c1.area;
            double a2 = c2.area;

            combinedContours.clear();
            final double SIZE_CUTOFF = 500;

            if (a0 >= SIZE_CUTOFF && a1 >= SIZE_CUTOFF && a2 < SIZE_CUTOFF) {
                System.out.println("a01");
                combinedContours.add(c0);
                combinedContours.add(c1);
            } else if (a0 < SIZE_CUTOFF && a1 >= SIZE_CUTOFF && a2 >= SIZE_CUTOFF) {
                System.out.println("a12");
                combinedContours.add(c1);
                combinedContours.add(c2);
            } else if (a0 >= SIZE_CUTOFF && a1 < SIZE_CUTOFF && a2 >= SIZE_CUTOFF) {
                System.out.println("a02");
                combinedContours.add(c0);
                combinedContours.add(c2);
            } else if (a0 >= SIZE_CUTOFF && a1 >= SIZE_CUTOFF && a2 >= SIZE_CUTOFF) {
                double dist01 = contourMinDist(c0.contour, c1.contour);
                double dist12 = contourMinDist(c1.contour, c2.contour);
                double dist02 = contourMinDist(c0.contour, c2.contour);
                if (dist01 < dist12 && dist01 < dist02) {
                    System.out.println("b01");
                    combinedContours.add(c0);
                    combinedContours.add(c1);
                } else if (dist12 < dist01 && dist12 < dist02) {
                    System.out.println("b12");
                    combinedContours.add(c1);
                    combinedContours.add(c2);
                } else {
                    System.out.println("b02");
                    combinedContours.add(c0);
                    combinedContours.add(c2);
                }
            } else {
                Log.i(TAG, "No gear target found: contours too small");
                Communications.root.putString("Status", "contours too small");
                releaseList0(combinedContours);
                contourCopy.release();
                return false;
            }
        }

        // make sure 0 is on the left
        double cx0 = centerOfContour(combinedContours.get(0).contour).x;
        double cx1 = centerOfContour(combinedContours.get(1).contour).x;
        if (cx0 > cx1) {
            ContourInfo tmp = combinedContours.remove(0);
            combinedContours.add(tmp);
        }

        MatOfPoint c0 = combinedContours.get(0).contour;
        MatOfPoint c1 = combinedContours.get(1).contour;

        // fit a 4-sided shape to each contour
        MatOfPoint2f d0 = new MatOfPoint2f();
        MatOfPoint2f d1 = new MatOfPoint2f();
        c0.convertTo(d0, CvType.CV_32FC2);
        c1.convertTo(d1, CvType.CV_32FC2);

        MatOfPoint2f e0 = quadFit(d0);
        MatOfPoint2f e1 = quadFit(d1);

        if (e0 == null || e1 == null) {
            Log.i(TAG, "No gear target found: couldn't quad fit");
            Communications.root.putString("Status", "failed to quad fit");
            releaseList0(combinedContours);
            contourCopy.release();
            c0.release();
            c1.release();
            d0.release();
            d1.release();
            if (e0 != null) e0.release();
            if (e1 != null) e1.release();
            return false;
        }

        // reorder clockwise from bottom left corner for left target, counterclockwise from bottom
        // right for right target. So we know which points we're talking about when we index into
        // the array
        reorderQuad(e0, true);
        reorderQuad(e1, false);
        MatOfPoint2f[] quads = new MatOfPoint2f[]{e0, e1};

        // we need the bottom outer corner of each target to construct the target quadrilateral. If
        // it's being blocked, use the slope of the unblocked target bottom edge to approximate it
        int potentialCornerCutContourIdx = -1;
        if (combinedContours.get(0).numSourceContours >= 2) {
            potentialCornerCutContourIdx = 0;
        } else if (combinedContours.get(1).numSourceContours >= 2) {
            potentialCornerCutContourIdx = 1;
        }

        if (potentialCornerCutContourIdx > -1) {
            Point[] qCut = quads[potentialCornerCutContourIdx].toArray();
            Point[] qUncut = quads[1 - potentialCornerCutContourIdx].toArray();

            double slopeU3U0 = calcSlope(qUncut[3].y - qUncut[0].y, qUncut[3].x - qUncut[0].x);
            double slopeC3C0 = calcSlope(qCut[3].y - qCut[0].y, qCut[3].x - qCut[0].x);
            if (Math.abs(slopeU3U0 - slopeC3C0) > 0.2) {
                System.out.println("fixing " + potentialCornerCutContourIdx);
                double mC10 = calcSlope(qCut[1].y - qCut[0].y, qCut[1].x - qCut[0].x);
                double bC10 = qCut[0].y - mC10 * qCut[0].x;
                double mU30 = slopeU3U0;
                double bU30 = qCut[3].y - mU30 * qCut[3].x;

                double intersectionX = (bU30 - bC10) / (mC10 - mU30);
                double intersectionY = mC10 * intersectionX + bC10;

                // screwed up calculations? Ignore it
                if (Double.isNaN(intersectionX) || Double.isNaN(intersectionY)
                        || Double.isInfinite(intersectionX) || Double.isInfinite(intersectionY)) {
                    Communications.root.putString("Status", "failed to correct missing corner");
                    releaseList0(combinedContours);
                    contourCopy.release();
                    c0.release();
                    c1.release();
                    d0.release();
                    d1.release();
                    e0.release();
                    e1.release();
                    return false;
                }

                qCut[0].x = intersectionX;
                qCut[0].y = intersectionY;
                quads[potentialCornerCutContourIdx].fromArray(qCut);
            }
        }

        // yay we've found a target 🎉
        Point[] p0 = e0.toArray();
        Point[] p1 = e1.toArray();
        Point[] bigQuad = new Point[]{p0[0], p0[1], p1[1], p1[0]};
        polyFit.fromArray(bigQuad);

        // clean up those pesky native resources
        releaseList0(combinedContours);
        contourCopy.release();
        c0.release();
        c1.release();
        d0.release();
        d1.release();
        e0.release();
        e1.release();

        return true;
    }

    private boolean findBoilerTarget(Mat src) {
        // filter out green
        Core.inRange(src, mFilterColorRange.getLower(),
                mFilterColorRange.getUpper(), src);

        releaseList(contours);

        // detect all contours
        Mat contourCopy = src.clone();
        Imgproc.findContours(contourCopy, contours, hierarchy, Imgproc.RETR_CCOMP,
                Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.size() < 2) {
            contourCopy.release();
            Communications.root.putString("Status", "<2 contours");
            return false;
        }

        List<ContourInfo> contourInfos = new LinkedList<>();
        for (MatOfPoint contour : contours) {
            contourInfos.add(new ContourInfo(contour, Imgproc.contourArea(contour), 1));
        }

        Collections.sort(contourInfos, new Comparator<ContourInfo>() {
            @Override
            public int compare(ContourInfo o1, ContourInfo o2) {
                return (int) Math.signum(o2.area - o1.area);
            }
        });

        ContourInfo c0 = contourInfos.get(0);
        ContourInfo c1 = contourInfos.get(1);

        if (centerOfContour(c0.contour).y > centerOfContour(c1.contour).y) {
            ContourInfo tmp = c0;
            c0 = c1;
            c1 = tmp;
        }

        Communications.root.putString("Region_Areas", String.format("%f%n%f", c0.area, c1.area));

        // make sure we have the right thing

        if (c0.area < 200 || c1.area < 200) {
            Log.i(TAG, "Didn't find correct size of targets");
            Communications.root.putString("Status", "contours too small");
            return false;
        }

        double areaRatio = c1.area / c0.area;
//        if (areaRatio < 0.40 || areaRatio > 0.60) {
//            Log.i(TAG, "Didn't find correct pair of targets: " + areaRatio);
//            Communications.root.putString("Status", "bad area ratio");
//            return false;
//        }

        // check if the entire contour is reasonably within bounds, otherwise we risk garbage
        // values from a clipped target
        List<Point> allPts = new ArrayList<>();
        allPts.addAll(c0.contour.toList());
        allPts.addAll(c1.contour.toList());
        for (Point point : allPts) {
            double x = point.x;
            double y = point.y;
            if (x < 2 || x > src.width() - 2 || y < 2 || y > src.height() - 2) {
                Log.i(TAG, "Target clipped");
                Communications.root.putString("Status", "clipped target");
                return false;
            }
        }

        MatOfInt hullIdx0 = new MatOfInt();
        MatOfInt hullIdx1 = new MatOfInt();
        Imgproc.convexHull(c0.contour, hullIdx0);
        Imgproc.convexHull(c1.contour, hullIdx1);
        c0.contour = normalizeHull(c0.contour, hullIdx0);
        c1.contour = normalizeHull(c1.contour, hullIdx1);

        MatOfPoint2f d0 = new MatOfPoint2f();
        MatOfPoint2f d1 = new MatOfPoint2f();
        c0.contour.convertTo(d0, CvType.CV_32FC2);
        c1.contour.convertTo(d1, CvType.CV_32FC2);

        RotatedRect r0 = Imgproc.minAreaRect(d0);
        RotatedRect r1 = Imgproc.minAreaRect(d1);

        Point[] box0 = new Point[4];
        r0.points(box0);
        Point[] box1 = new Point[4];
        r1.points(box1);

        Comparator<Point> comp = new Comparator<Point>() {
            @Override
            public int compare(Point p0, Point p1) {
                return Double.compare(-p0.x, -p1.x);
            }
        };

        Arrays.sort(box0, comp);
        Arrays.sort(box1, comp);

        if (box0[0].y > box0[1].y) {
            Point tmp = box0[0];
            box0[0] = box0[1];
            box0[1] = tmp;
        }
        if (box1[0].y > box1[1].y) {
            Point tmp = box1[0];
            box1[0] = box1[1];
            box1[1] = tmp;
        }

        polyFit.fromArray(box1[0], box0[0], box0[1], box1[1]);
        return true;
    }

    static double calcSlope(double dy, double dx) {
        if (dx != 0) {
            return dy / dx;
        } else {
            return Double.MAX_VALUE;
        }
    }

    static double contourMinDist(MatOfPoint c0, MatOfPoint c1) {
        Point[] p0 = c0.toArray();
        Point[] p1 = c1.toArray();
        double minSqDist = Double.MAX_VALUE;
        for (int i = 0; i < p0.length; i++) {
            for (int j = 0; j < p1.length; j++) {
                double sqDist = squaredDist(p0[i], p1[j]);
                if (sqDist < minSqDist) {
                    minSqDist = sqDist;
                }
            }
        }
        return minSqDist;
    }

    static double squaredDist(Point p0, Point p1) {
        double dx = p0.x - p1.x;
        double dy = p0.y - p1.y;
        return dx * dx + dy * dy;
    }

    static Point centerOfContour(MatOfPoint contour) {
        Moments m = Imgproc.moments(contour);
        return new Point(m.m10 / m.m00, m.m01 / m.m00);
    }

    static Point centerOfContour(MatOfPoint2f contour) {
        Moments m = Imgproc.moments(contour);
        return new Point(m.m10 / m.m00, m.m01 / m.m00);
    }

    static void drawMatOfPoint2f(Mat dst, MatOfPoint2f pts, Scalar color) {
        Point[] ptsArray = pts.toArray();
        for (int i = 0; i < ptsArray.length; ++i) {
            Imgproc.line(dst, ptsArray[i], ptsArray[(i + 1) % ptsArray.length], color);
            Imgproc.putText(dst, Integer.toString(i), ptsArray[i], Core.FONT_HERSHEY_PLAIN, 1,
                    new Scalar(0, 0, 255));
        }
    }

    static void reorderQuad(MatOfPoint2f polyFit, boolean clockwise) {
        Point[] polyFitPoint = polyFit.toArray();
        Arrays.sort(polyFitPoint, new Comparator<Point>() {
            @Override
            public int compare(org.opencv.core.Point point1,
                               org.opencv.core.Point point2) {
                return Double.compare(point1.x, point2.x);
            }
        });

        Point left1 = polyFitPoint[0], left2 = polyFitPoint[1],
                right1 = polyFitPoint[2], right2 = polyFitPoint[3];
        if (left1.y < left2.y) {
            polyFitPoint[0] = left2;
            polyFitPoint[1] = left1;
        }
        if (right1.y > right2.y) {
            polyFitPoint[2] = right2;
            polyFitPoint[3] = right1;
        }

        if (!clockwise) {
            Point tmp = polyFitPoint[0];
            polyFitPoint[0] = polyFitPoint[3];
            polyFitPoint[3] = tmp;
            tmp = polyFitPoint[1];
            polyFitPoint[1] = polyFitPoint[2];
            polyFitPoint[2] = tmp;
        }

        polyFit.fromArray(polyFitPoint);
    }

    static MatOfPoint2f quadFit(MatOfPoint2f src) {
        MatOfPoint2f polyFit = new MatOfPoint2f();
        // play with epsilon until we get a quadrangle
        // as we oscillate around the goal of 4 points, converge
        // epsilon by multiplying delta-epsilon by 0.9
        double epsilon = 0;
        double deltaEpsilon = 0.1;
        int oscPhase = 0;
        int loops = 0, maxLoops = 10000;
        do {
            loops++;
            if (loops > maxLoops) {
                polyFit = null;
                break;
            }
            Imgproc.approxPolyDP(src, polyFit, epsilon, true);
            if (polyFit.total() < 4) {
                epsilon += deltaEpsilon;
                if (oscPhase == 0) {
                    oscPhase = 1;
                    deltaEpsilon *= 0.9;
                }
            } else {
                epsilon -= deltaEpsilon;
                if (oscPhase == 1) {
                    oscPhase = 0;
                    deltaEpsilon *= 0.9;
                }
            }
        } while (polyFit.total() != 4);

        return polyFit;
    }

    static MatOfPoint normalizeHull(MatOfPoint src, MatOfInt hull) {
        MatOfPoint mopOut = new MatOfPoint();
        mopOut.create((int) hull.size().height, 1, CvType.CV_32SC2);

        for (int i = 0; i < hull.size().height; i++) {
            int index = (int) hull.get(i, 0)[0];
            double[] point = new double[]{src.get(index, 0)[0], src.get(index, 0)[1]};
            mopOut.put(i, 0, point);
        }
        return mopOut;
    }

    static double calcLength(Point[] line) {
        double dx = line[0].x - line[1].x;
        double dy = line[0].y - line[1].y;
        return dx * dx + dy * dy;
    }
}

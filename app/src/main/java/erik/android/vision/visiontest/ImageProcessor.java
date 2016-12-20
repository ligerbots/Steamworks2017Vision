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
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class ImageProcessor implements Runnable {
    private static final String TAG = "ImageProcessor";

    private Thread mProcessingThread;

    private final Object mProcessingLock = new Object();
    private Mat mProcessingHsvMat;
    private MatOfPoint mProcessingLargestContour = null;
    private Point[] mProcessingPolyFit = null;

    private NTColorPicker mFilterColorRange;
    private FpsCounter mProcessingFps;
    private Calibration mCalibration;

    public ImageProcessor(Calibration calibration) {
        mFilterColorRange = new NTColorPicker("Vision/colorRange", NTColorPicker.ColorMode.HSV);
        mProcessingFps = new FpsCounter("ProcessingThread");
        mCalibration = calibration;

        mProcessingThread = new Thread(this);
        mProcessingThread.setName("OpenCV Image Processor");
        mProcessingThread.setDaemon(true);
        mProcessingThread.start();
    }

    public void postUpdateAndDraw(Mat sourceBgrMat, Mat drawingRgbMat) {
        synchronized(mProcessingLock) {
            if(mProcessingThread.getState() == Thread.State.WAITING) {
                if(mProcessingHsvMat == null) {
                    mProcessingHsvMat = new Mat();
                }
                Imgproc.cvtColor(sourceBgrMat, mProcessingHsvMat, Imgproc.COLOR_RGB2HSV);
                mProcessingLock.notify();
            }

            if(mProcessingLargestContour != null) {
                Imgproc.drawContours(drawingRgbMat,
                        Collections.singletonList(mProcessingLargestContour), 0,
                        new Scalar(255, 0, 0));
            }
            if(mProcessingPolyFit != null) {
                org.opencv.core.Point[] localCopy = mProcessingPolyFit;
                for(int i=0; i<localCopy.length; ++i) {
                    Imgproc.line(drawingRgbMat, localCopy[i], localCopy[(i+1)%localCopy.length],
                            new Scalar(0, 255, 0));
                    Imgproc.putText(drawingRgbMat, Integer.toString(i), localCopy[i],
                            Core.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 0, 255));
                }
            }
        }
    }

    @Override
    public void run() {
        try {
            //noinspection InfiniteLoopStatement
            while (true) {
                Log.i(TAG, "Processing thread waiting");
                synchronized (mProcessingLock) {
                    try {
                        mProcessingLock.wait();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                Log.i(TAG, "Processing thread loop");
                long st = System.currentTimeMillis();
                // send image histogram
                mFilterColorRange.setHistogram(mProcessingHsvMat);
                // Filter HSV by color specified by seek bar
                Core.inRange(mProcessingHsvMat, mFilterColorRange.getLower(),
                        mFilterColorRange.getUpper(), mProcessingHsvMat);
                // smoothen it out
                //Imgproc.medianBlur(mProcessingHsvMat, mProcessingHsvMat, 11);

                // find contours
                List<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();
                Mat contourCopy = mProcessingHsvMat.clone();
                Imgproc.findContours(contourCopy, contours, hierarchy, Imgproc.RETR_CCOMP,
                        Imgproc.CHAIN_APPROX_SIMPLE);

                // search for largest contour
                double largestArea = 0;
                MatOfPoint largestContour = null;
                for (int i = 0; i < contours.size(); i++) {
                    double a = Imgproc.contourArea(contours.get(i));
                    if (a > largestArea) {
                        largestArea = a;
                        largestContour = contours.get(i);
                    }
                }

                MatOfPoint2f polyFit = new MatOfPoint2f();
                if (largestContour != null) {
                    MatOfInt hull = new MatOfInt();
                    Imgproc.convexHull(largestContour, hull);

                    // convert contour to its convex hull
                    MatOfPoint mopOut = new MatOfPoint();
                    mopOut.create((int) hull.size().height, 1, CvType.CV_32SC2);

                    for (int i = 0; i < hull.size().height; i++) {
                        int index = (int) hull.get(i, 0)[0];
                        double[] point = new double[]{
                                largestContour.get(index, 0)[0], largestContour.get(index, 0)[1]
                        };
                        mopOut.put(i, 0, point);
                    }

                    largestContour = mopOut;

                    // convert contour to MatOfPoint2f for approxPolyDP
                    MatOfPoint2f contour2f = new MatOfPoint2f(largestContour.toArray());

                    // play with epsilon until we get a quadrangle
                    // as we oscillate around the goal of 4 points, converge
                    // epsilon by multiplying delta-epsilon by 0.9
                    double epsilon = 0;
                    double deltaEpsilon = 0.1;
                    int oscPhase = 0;
                    int loops = 0, maxLoops = 10000;
                    Log.i(TAG, "Processing thread starting quad fit");
                    do {
                        loops++;
                        if(loops > maxLoops) {
                            polyFit = null;
                            Log.w(TAG, "Processing thread quad fit giving up");
                            break;
                        }
                        Imgproc.approxPolyDP(contour2f, polyFit, epsilon, true);

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
                    Log.i(TAG, "Processing thread quad fit done");

                    if(polyFit != null) {
                        // reorder to clockwise order from bottom left
                        //  1 /----------\ 2
                        //    |          |
                        //  0 \----------/ 3
                        org.opencv.core.Point[] polyFitPoint = polyFit.toArray();
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

                        polyFit.fromArray(polyFitPoint);

                        // calculate camera transform
                        MatOfPoint3f objPoints = new MatOfPoint3f();
                        objPoints.fromArray(
                                new Point3(0, 0, 0),
                                new Point3(0, 8, 0),
                                new Point3(12, 8, 0),
                                new Point3(12, 0, 0)
                        );

                        double[] cameraMatrixA = mCalibration.getCameraMatrixArray();
                        double[] distortionCoefficientsA =
                                mCalibration.getDistortionCoefficientsArray();

                        if(cameraMatrixA.length > 0 && distortionCoefficientsA.length > 0) {
                            Mat cameraMatrix = new Mat();
                            MatOfDouble distortCoeff = new MatOfDouble();
                            Mat.eye(3, 3, CvType.CV_64FC1).copyTo(cameraMatrix);
                            cameraMatrix.put(0, 0, 1.0);
                            Mat.zeros(5, 1, CvType.CV_64FC1).copyTo(distortCoeff);
                            Mat rvec = new Mat(), tvec = new Mat();

                            for(int i = 0; i < 3; i++){
                                for(int j = 0; j < 3; j++) {
                                    cameraMatrix.put(i, j, cameraMatrixA[i * 3 + j]);
                                }
                            }
                            for(int i = 0; i < distortionCoefficientsA.length; i++){
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
                                            (float) Math.sqrt(rmat.get(1, 2)[0]*rmat.get(1, 2)[0]
                                                    + rmat.get(2, 2)[0]*rmat.get(2, 2)[0])),
                                    Core.fastAtan2((float) rmat.get(1, 0)[0],
                                            (float) rmat.get(0, 0)[0])
                            );

                            double[] rvecDouble = new double[(int) rvec.total()];
                            rvec.get(0, 0, rvecDouble);
                            double[] tvecDouble = new double[(int) tvec.total()];
                            tvec.get(0, 0, tvecDouble);
                            NetworkTable result = NetworkTable.getTable("Vision/result");
                            result.putNumber("rvec_pitch", rvecDouble[0]);
                            result.putNumber("rvec_yaw", rvecDouble[1]);
                            result.putNumber("rvec_roll", rvecDouble[2]);
                            result.putNumber("tvec_x", tvecDouble[0]);
                            result.putNumber("tvec_y", tvecDouble[1]);
                            result.putNumber("tvec_z", tvecDouble[2]);
                            Communications.dataServerSendData(rvecDouble[0], rvecDouble[1],
                                    rvecDouble[2], tvecDouble[0], tvecDouble[1], tvecDouble[2]);
                        }
                    }
                }

                Log.i(TAG, "Processing thread dt: " + (System.currentTimeMillis() - st));

                mProcessingLargestContour = largestContour;
                if(polyFit != null) {
                    mProcessingPolyFit = polyFit.toArray();
                } else {
                    mProcessingPolyFit = null;
                }

                mProcessingFps.feed();
                Log.i(TAG, "Processing thread loop done");
            }
        } catch(Throwable t) {
            Log.e(TAG, "Processing thread error!", t);
        }
    }
}

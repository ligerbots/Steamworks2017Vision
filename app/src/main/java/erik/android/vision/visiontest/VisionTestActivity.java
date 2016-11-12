package erik.android.vision.visiontest;

import android.content.Intent;
import android.graphics.Rect;
import android.os.Bundle;
import android.os.Environment;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.SeekBar;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.OpenCVLoader;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import erik.android.vision.visiontest.calibration.CalibrationResult;
import erik.android.vision.visiontest.calibration.CameraCalibrationActivity;

public class VisionTestActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener2 {
    public static final String TAG = "VisionTestActivity";
    public static final int CS_CONTROL_PORT = 5809;
    public static final int CS_STREAM_PORT = 5810;
    public static final byte[] CS_MAGIC_NUMBER = new byte[]{1, 0, 0, 0};
    public static InetAddress CS_BROADCAST_ADDRESS;

    static {
        if (!OpenCVLoader.initDebug()) {
            Log.e(TAG, "OpenCV failed to load");
        }
    }

    private CameraBridgeViewBase mOpenCvCameraView;
    private DatagramSocket udpCameraServerSocket;
    private long lastSendTime = 0;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_vision_test);
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.HelloOpenCvView);
        mOpenCvCameraView.setMaxFrameSize(480, 360);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);

        NetworkTable.setClientMode();
        NetworkTable.setDSClientEnabled(false);
        NetworkTable.setIPAddress("Erik-PC");
        NetworkTable.initialize();
        NetworkTable.getTable("/").putString("AndroidStatus", "Hello, World!");

        try {
            CS_BROADCAST_ADDRESS = InetAddress.getByName("255.255.255.255");
            udpCameraServerSocket = new DatagramSocket(CS_CONTROL_PORT);
            udpCameraServerSocket.setReuseAddress(true);
            udpCameraServerSocket.setBroadcast(true);
        } catch(Exception e) {
            Log.e(TAG, "UDP error", e);
        }
    }

    @Override
    public void onResume() {
        super.onResume();

        if(cameraMatrix != null){
            cameraMatrix.release();
        }
        if(distortCoeff != null) {
            distortCoeff.release();
        }
        cameraMatrix = new Mat();
        distortCoeff = new MatOfDouble();
        Mat.eye(3, 3, CvType.CV_64FC1).copyTo(cameraMatrix);
        cameraMatrix.put(0, 0, 1.0);
        Mat.zeros(5, 1, CvType.CV_64FC1).copyTo(distortCoeff);
        boolean success = CalibrationResult.tryLoad(this, cameraMatrix, distortCoeff);
        if(!success || cameraMatrix.rows() != 3){
            cameraMatrix.release();
            distortCoeff.release();
            cameraMatrix = null;
            distortCoeff = null;
            Log.i(TAG, "Couldn't load camera parameters");
        }

        if (mOpenCvCameraView != null)
            mOpenCvCameraView.enableView();
    }

    @Override
    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        NetworkTable.shutdown();
    }

    Scalar markColor1, markColor2;
    String saveFile;
    boolean doNextSave = false;
    Mat cameraMatrix;
    MatOfDouble distortCoeff;

    public void onCameraViewStarted(int width, int height) {
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        String filename = "vision test.png";
        File file = new File(path, filename);
        saveFile = file.toString();

        markColor1 = new Scalar(0, 255, 0);
        markColor2 = new Scalar(0, 0, 255);
    }

    public void onCameraViewStopped() {
    }

    public void onCalibClick(View v){
        Intent intent = new Intent(this, CameraCalibrationActivity.class);
        startActivity(intent);
    }

    public void onSaveClick(View v) {
        doNextSave = true;
    }

    protected float lastTouchX, lastTouchY;

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        // map touch to image location based on the image rects and
        // position of the surface widget on screen
        int[] location = new int[2];
        mOpenCvCameraView.getLocationOnScreen(location);
        float screenX = event.getRawX();
        float screenY = event.getRawY();
        // map from screen to Surface coordinates
        float viewMappedX = screenX - location[0];
        float viewMappedY = screenY - location[1];

        Rect src = mOpenCvCameraView.getSrcRect();
        Rect dst = mOpenCvCameraView.getDstRect();

        // map from Surface to image coordinates
        lastTouchX = ((viewMappedX - dst.left) / (dst.width())) * src.width();
        lastTouchY = ((viewMappedY - dst.top) / (dst.height())) * src.height();

        Log.i(TAG, "Touch");

        return super.onTouchEvent(event);
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        // Make sure we don't crash the phone with the resources we're not cleaning up
        System.gc();

        //// 1. Setup
        Mat originalFrame = inputFrame.rgba();
        if (doNextSave) {
            doNextSave = false;
            Imgcodecs.imwrite(saveFile, originalFrame);
        }

        // Create copy for processing
        Mat procFrame = originalFrame.clone();

        // Convert to HSV for filtering
        Imgproc.cvtColor(procFrame, procFrame, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(procFrame, procFrame, Imgproc.COLOR_RGB2HSV);

        double[] pixel = procFrame.get((int) lastTouchX, (int) lastTouchY);

        //// 2. Processing
        if (pixel != null) {
            int prog = ((SeekBar) findViewById(R.id.seekBar)).getProgress();

            // Filter HSV by color specified by seek bar
            Core.inRange(procFrame, new Scalar(0, 0, 255 - prog), new Scalar(255, prog, 255), procFrame);
            // smoothen it out
            Imgproc.medianBlur(procFrame, procFrame, 11);

            // find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Mat contourCopy = procFrame.clone();
            Imgproc.findContours(contourCopy, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

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

            // calculate bounding rectangle, convex hull, and fitted quadrangle
            org.opencv.core.Rect boundingRect = null;
            MatOfPoint2f polyFit = new MatOfPoint2f();
            int cmX = 0, cmY = 0;
            Mat rvec = new Mat(), tvec = new Mat();
            if (largestContour != null && cameraMatrix != null && distortCoeff != null) {
                MatOfInt hull = new MatOfInt();
                Imgproc.convexHull(largestContour, hull);

                // convert contour to its convex hull
                MatOfPoint mopOut = new MatOfPoint();
                mopOut.create((int)hull.size().height,1, CvType.CV_32SC2);

                for(int i = 0; i < hull.size().height ; i++)
                {
                    int index = (int)hull.get(i, 0)[0];
                    double[] point = new double[] {
                            largestContour.get(index, 0)[0], largestContour.get(index, 0)[1]
                    };
                    mopOut.put(i, 0, point);
                }

                largestContour = mopOut;

                boundingRect = Imgproc.boundingRect(largestContour);

                // convert contour to MatOfPoint2f for approxPolyDP
                MatOfPoint2f contour2f = new MatOfPoint2f(largestContour.toArray());

                // play with epsilon until we get a quadrangle
                // as we oscillate around the goal of 4 points, converge
                // epsilon by multiplying delta-epsilon by 0.9
                double epsilon = 0;
                double deltaEpsilon = 0.1;
                int oscPhase = 0;
                do {
                    Imgproc.approxPolyDP(contour2f, polyFit, epsilon, true);

                    if(polyFit.total() < 4){
                        epsilon += deltaEpsilon;
                        if(oscPhase == 0){
                            oscPhase = 1;
                            deltaEpsilon *= 0.9;
                        }
                    } else {
                        epsilon -= deltaEpsilon;
                        if(oscPhase == 1){
                            oscPhase = 0;
                            deltaEpsilon *= 0.9;
                        }
                    }

                } while(polyFit.total() != 4);

                // reorder to clockwise order from bottom left
                //  1 /----------\ 2
                //    |          |
                //  0 \----------/ 3
                Point[] polyFitPoint = polyFit.toArray();
                Arrays.sort(polyFitPoint, new Comparator<Point>() {
                    @Override
                    public int compare(Point point1, Point point2) {
                        return Double.compare(point1.x, point2.x);
                    }
                });

                Point left1 = polyFitPoint[0], left2 = polyFitPoint[1],
                        right1 = polyFitPoint[2], right2 = polyFitPoint[3];
                if(left1.y < left2.y){
                    polyFitPoint[0] = left2;
                    polyFitPoint[1] = left1;
                }
                if(right1.y > right2.y){
                    polyFitPoint[2] = right2;
                    polyFitPoint[3] = right1;
                }

                polyFit.fromArray(polyFitPoint);

                // calculate camera transform
                MatOfPoint3f objPoints = new MatOfPoint3f();
                objPoints.fromArray(
                        new Point3(0, 8.5, 0),
                        new Point3(0, 0, 0),
                        new Point3(11, 0, 0),
                        new Point3(11, 8.5, 0)
                );

                Calib3d.solvePnP(objPoints, polyFit, cameraMatrix, distortCoeff, rvec, tvec);

                Mat rmat = new Mat();
                Calib3d.Rodrigues(rvec, rmat);
                rvec.release();
                rvec = new Mat(1, 3, CvType.CV_64F);
                rvec.put(0, 0,
                        Core.fastAtan2((float) rmat.get(1, 2)[0], (float) rmat.get(2, 2)[0]),
                        Core.fastAtan2((float) -rmat.get(2, 0)[0],
                                (float) Math.sqrt(rmat.get(1, 2)[0]*rmat.get(1, 2)[0] + rmat.get(2, 2)[0]*rmat.get(2, 2)[0])),
                        Core.fastAtan2((float) rmat.get(1, 0)[0], (float) rmat.get(0, 0)[0])
                );

                // calculate center of mass
                Moments moments = Imgproc.moments(largestContour);
                cmX = (int) (moments.get_m10() / moments.get_m00());
                cmY = (int) (moments.get_m01() / moments.get_m00());
            }

            //// 3. Display

            // convert to RGBA so we can merge the frames
            Imgproc.cvtColor(procFrame, procFrame, Imgproc.COLOR_GRAY2RGBA);
            // do merge
            Core.addWeighted(procFrame, 0.5, originalFrame, 0.5, 0, originalFrame);

            // draw the markup
            Point touch = new Point((int) lastTouchX, (int) lastTouchY);
            Imgproc.circle(originalFrame, touch, 10, markColor1);
            if (boundingRect != null) {
                Imgproc.drawContours(originalFrame, Collections.singletonList(largestContour), 0, markColor2);
                Imgproc.circle(originalFrame, new Point(cmX, cmY), 10, markColor1);

                double[] rvecDouble = new double[(int) rvec.total()];
                rvec.get(0, 0, rvecDouble);
                double[] tvecDouble = new double[(int) tvec.total()];
                tvec.get(0, 0, tvecDouble);

                StringBuilder rvecStr = new StringBuilder(), tvecStr = new StringBuilder();
                for (double aRvecDouble : rvecDouble) {
                    rvecStr.append(String.format(Locale.getDefault(), "%3.1f, ", aRvecDouble));
                }
                for (double aTvecDouble : tvecDouble) {
                    tvecStr.append(String.format(Locale.getDefault(), "%3.1f, ", aTvecDouble));
                }

                Imgproc.putText(originalFrame, rvecStr.toString(), new Point(cmX - 40, cmY + 20), Core.FONT_HERSHEY_PLAIN, 1, markColor2);
                Imgproc.putText(originalFrame, tvecStr.toString(), new Point(cmX - 40, cmY + 40), Core.FONT_HERSHEY_PLAIN, 1, markColor2);

                Imgproc.rectangle(originalFrame, boundingRect.tl(), boundingRect.br(), markColor1);

                Point points[] = polyFit.toArray();

                for(int i=0; i<points.length; ++i) {
                    Imgproc.line(originalFrame, points[i], points[(i+1)%points.length], markColor1);
                    Imgproc.putText(originalFrame, Integer.toString(i), points[i], Core.FONT_HERSHEY_PLAIN, 1, markColor2);
                }
            }
            Imgproc.putText(originalFrame, Arrays.toString(pixel), touch, Core.FONT_HERSHEY_PLAIN, 1, markColor2);
            String status = "Ready";
            if(cameraMatrix == null || distortCoeff == null){
                status = "No calibration parameters";
            }
            status += " | ";
            if(NetworkTable.connections().length > 0) {
                status += "NT connected";
            } else {
                status += "NT not connected";
            }
            Imgproc.putText(originalFrame, status, new Point(0, 10), Core.FONT_HERSHEY_PLAIN, 1, markColor2);
        }

        if(System.currentTimeMillis() - lastSendTime > 100) {
            MatOfByte output = new MatOfByte();
            Mat copyFrame = new Mat();
            Imgproc.cvtColor(originalFrame, copyFrame, Imgproc.COLOR_RGBA2BGR);
            Imgcodecs.imencode(".jpg", copyFrame, output);
            byte[] data = output.toArray();
            int length = data.length;
            byte[] lengthByte = new byte[]{
                    (byte) (length >>> 24),
                    (byte) (length >>> 16),
                    (byte) (length >>> 8),
                    (byte) (length)
            };
            byte[] packet = new byte[data.length + lengthByte.length + CS_MAGIC_NUMBER.length];
            System.arraycopy(CS_MAGIC_NUMBER, 0, packet, 0, CS_MAGIC_NUMBER.length);
            System.arraycopy(lengthByte, 0, packet, CS_MAGIC_NUMBER.length, lengthByte.length);
            System.arraycopy(data, 0, packet, CS_MAGIC_NUMBER.length + lengthByte.length, data.length);
            DatagramPacket pkt = new DatagramPacket(packet, packet.length, CS_BROADCAST_ADDRESS, CS_STREAM_PORT);
            try {
                udpCameraServerSocket.send(pkt);
            } catch (IOException e) {
                Log.e(TAG, "UDP send error", e);
            }

            lastSendTime = System.currentTimeMillis();
        }

        return originalFrame;
    }
}

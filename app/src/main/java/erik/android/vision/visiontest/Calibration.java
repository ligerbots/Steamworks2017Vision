package erik.android.vision.visiontest;

import android.util.Log;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

public class Calibration {
    private static final String TAG = "Calibration";
    private static final String TABLE_NAME = "Vision/Calibration";

    private final Size mPatternSize = new Size(4, 11);
    private final int mCornersSize = (int)(mPatternSize.width * mPatternSize.height);
    private boolean mPatternWasFound = false;
    private MatOfPoint2f mCorners = new MatOfPoint2f();
    private List<Mat> mCornersBuffer = new ArrayList<Mat>();
    private boolean mIsCalibrated = false;

    private Mat mCameraMatrix = new Mat();
    private Mat mDistortionCoefficients = new Mat();
    private int mFlags;
    private double mRms;
    private Size mImageSize;

    private ITable mTable;

    public Calibration(int width, int height) {
        mImageSize = new Size(width, height);
        mFlags = Calib3d.CALIB_FIX_PRINCIPAL_POINT +
                 Calib3d.CALIB_ZERO_TANGENT_DIST +
                 Calib3d.CALIB_FIX_ASPECT_RATIO +
                 Calib3d.CALIB_FIX_K4 +
                 Calib3d.CALIB_FIX_K5;
        Mat.eye(3, 3, CvType.CV_64FC1).copyTo(mCameraMatrix);
        mCameraMatrix.put(0, 0, 1.0);
        Mat.zeros(5, 1, CvType.CV_64FC1).copyTo(mDistortionCoefficients);
        Log.i(TAG, "Instantiated new " + this.getClass());

        mTable = NetworkTable.getTable(TABLE_NAME);

        mTable.putBoolean("enableCalibration", false);

        new NTCommand(TABLE_NAME + "/TakeFrame", "CalibrationTakeFrame", new Runnable() {
            @Override
            public void run() {
                if(mTable.getBoolean("enableCalibration", false)) {
                    addCorners();
                }
            }
        });

        new NTCommand(TABLE_NAME + "/Calibrate", "CalibrationCalibrate", new Runnable() {
            @Override
            public void run() {
                if(mTable.getBoolean("enableCalibration", false)) {
                    calibrate((float) mTable.getNumber("squareSize", 0.669291));
                    Mat cameraMatrix = getCameraMatrix();
                    Mat distortionCoefficients = getDistortionCoefficients();
                    double[] cameraMatrixArray =
                            new double[cameraMatrix.rows() * cameraMatrix.cols()];
                    cameraMatrix.get(0, 0, cameraMatrixArray);
                    double[] distortionCoefficientsArray = new double[
                            distortionCoefficients.rows() * distortionCoefficients.cols()];
                    distortionCoefficients.get(0, 0, distortionCoefficientsArray);
                    mTable.putNumberArray("cameraMatrix", cameraMatrixArray);
                    mTable.putNumberArray("distortionCoefficients", distortionCoefficientsArray);
                    mTable.putNumberArray("calibrationResolution",
                            new double[]{mImageSize.width, mImageSize.height});
                }
            }
        });
    }

    public double[] getCameraMatrixArray() {
        return mTable.getNumberArray("cameraMatrix", new double[0]);
    }

    public double[] getDistortionCoefficientsArray() {
        return mTable.getNumberArray("distortionCoefficients", new double[0]);
    }

    public double[] getCalibrationResolutionArray() {
        return mTable.getNumberArray("calibrationResolution", new double[0]);
    }

    public boolean isEnabled() {
        return mTable.getBoolean("enableCalibration", false);
    }

    public void processFrame(Mat grayFrame, Mat rgbaFrame) {
        findPattern(grayFrame);
        renderFrame(rgbaFrame);
    }

    public void calibrate(float squareSize) {
        ArrayList<Mat> rvecs = new ArrayList<Mat>();
        ArrayList<Mat> tvecs = new ArrayList<Mat>();
        Mat reprojectionErrors = new Mat();
        ArrayList<Mat> objectPoints = new ArrayList<Mat>();
        objectPoints.add(Mat.zeros(mCornersSize, 1, CvType.CV_32FC3));
        calcBoardCornerPositions(objectPoints.get(0), squareSize);
        for (int i = 1; i < mCornersBuffer.size(); i++) {
            objectPoints.add(objectPoints.get(0));
        }

        Calib3d.calibrateCamera(objectPoints, mCornersBuffer, mImageSize,
                mCameraMatrix, mDistortionCoefficients, rvecs, tvecs, mFlags);

        mIsCalibrated = Core.checkRange(mCameraMatrix)
                && Core.checkRange(mDistortionCoefficients);

        //mRms = computeReprojectionErrors(objectPoints, rvecs, tvecs, reprojectionErrors);
        //Log.i(TAG, String.format("Average re-projection error: %f", mRms));
        Log.i(TAG, "Camera matrix: " + mCameraMatrix.dump());
        Log.i(TAG, "Distortion coefficients: " + mDistortionCoefficients.dump());
    }

    public void clearCorners() {
        mCornersBuffer.clear();
    }

    private void calcBoardCornerPositions(Mat corners, float squareSize) {
        final int cn = 3;
        float positions[] = new float[mCornersSize * cn];

        for (int i = 0; i < mPatternSize.height; i++) {
            for (int j = 0; j < mPatternSize.width * cn; j += cn) {
                positions[(int) (i * mPatternSize.width * cn + j + 0)] =
                        (2 * (j / cn) + i % 2) * (float) squareSize;
                positions[(int) (i * mPatternSize.width * cn + j + 1)] =
                        i * (float) squareSize;
                positions[(int) (i * mPatternSize.width * cn + j + 2)] = 0;
            }
        }
        corners.create(mCornersSize, 1, CvType.CV_32FC3);
        corners.put(0, 0, positions);
    }

    private double computeReprojectionErrors(List<Mat> objectPoints,
                                             List<Mat> rvecs, List<Mat> tvecs, Mat perViewErrors) {
        MatOfPoint2f cornersProjected = new MatOfPoint2f();
        double totalError = 0;
        double error;
        float viewErrors[] = new float[objectPoints.size()];

        MatOfDouble distortionCoefficients = new MatOfDouble(mDistortionCoefficients);
        int totalPoints = 0;
        for (int i = 0; i < objectPoints.size(); i++) {
            MatOfPoint3f points = new MatOfPoint3f(objectPoints.get(i));
            Calib3d.projectPoints(points, rvecs.get(i), tvecs.get(i),
                    mCameraMatrix, distortionCoefficients, cornersProjected);
            error = Core.norm(mCornersBuffer.get(i), cornersProjected, Core.NORM_L2);

            int n = objectPoints.get(i).rows();
            viewErrors[i] = (float) Math.sqrt(error * error / n);
            totalError  += error * error;
            totalPoints += n;
        }
        perViewErrors.create(objectPoints.size(), 1, CvType.CV_32FC1);
        perViewErrors.put(0, 0, viewErrors);

        return Math.sqrt(totalError / totalPoints);
    }

    private void findPattern(Mat grayFrame) {
        mPatternWasFound = Calib3d.findCirclesGrid(grayFrame, mPatternSize,
                mCorners, Calib3d.CALIB_CB_ASYMMETRIC_GRID);
    }

    public void addCorners() {
        if (mPatternWasFound) {
            mCornersBuffer.add(mCorners.clone());
        }
    }

    private void drawPoints(Mat rgbaFrame) {
        Calib3d.drawChessboardCorners(rgbaFrame, mPatternSize, mCorners, mPatternWasFound);
    }

    private void renderFrame(Mat rgbaFrame) {
        drawPoints(rgbaFrame);

        Imgproc.putText(rgbaFrame, "Captured: " + mCornersBuffer.size(),
                new Point(0, rgbaFrame.rows() * 0.1),
                Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 255, 0));
    }

    public Mat getCameraMatrix() {
        return mCameraMatrix;
    }

    public Mat getDistortionCoefficients() {
        return mDistortionCoefficients;
    }

    public int getCornersBufferSize() {
        return mCornersBuffer.size();
    }

    public double getAvgReprojectionError() {
        return mRms;
    }

    public boolean isCalibrated() {
        return mIsCalibrated;
    }

    public void setCalibrated() {
        mIsCalibrated = true;
    }
}

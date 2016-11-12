// This sample is based on "Camera calibration With OpenCV" tutorial:
// http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
//
// It uses standard OpenCV asymmetric circles grid pattern 11x4:
// https://github.com/Itseez/opencv/blob/2.4/doc/acircles_pattern.png.
// The results are the camera matrix and 5 distortion coefficients.
//
// Tap on highlighted pattern to capture pattern corners for calibration.
// Move pattern along the whole screen and capture data.
//
// When you've captured necessary amount of pattern corners (usually ~20 are enough),
// press "Calibrate" button for performing camera calibration.

package erik.android.vision.visiontest.calibration;

import android.app.Activity;
import android.app.ProgressDialog;
import android.content.res.Resources;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.WindowManager;
import android.widget.Toast;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

import erik.android.vision.visiontest.R;

public class CameraCalibrationActivity extends Activity implements CvCameraViewListener2, OnTouchListener {
    static {
        OpenCVLoader.initDebug();
    }

    private static final String TAG = "CalibrationActivity";

    private CameraBridgeViewBase mOpenCvCameraView;
    private CameraCalibrator mCalibrator;
    private OnCameraFrameRender mOnCameraFrameRender;
    private int mWidth;
    private int mHeight;

    public CameraCalibrationActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.camera_calibration_surface_view);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.camera_calibration_java_surface_view);
        mOpenCvCameraView.setMaxFrameSize(480, 360);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
    }

    @Override
    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume() {
        super.onResume();
        mOpenCvCameraView.enableView();
        mOpenCvCameraView.setOnTouchListener(CameraCalibrationActivity.this);
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void calibrateClick(View v) {
        final Resources res = getResources();
        if (mCalibrator.getCornersBufferSize() < 2) {
            (Toast.makeText(this, res.getString(R.string.more_samples), Toast.LENGTH_SHORT)).show();
            return;
        }

        mOnCameraFrameRender = new OnCameraFrameRender(new PreviewFrameRender());
        new AsyncTask<Void, Void, Void>() {
            private ProgressDialog calibrationProgress;

            @Override
            protected void onPreExecute() {
                calibrationProgress = new ProgressDialog(CameraCalibrationActivity.this);
                calibrationProgress.setTitle(res.getString(R.string.calibrating));
                calibrationProgress.setMessage(res.getString(R.string.please_wait));
                calibrationProgress.setCancelable(false);
                calibrationProgress.setIndeterminate(true);
                calibrationProgress.show();
            }

            @Override
            protected Void doInBackground(Void... arg0) {
                mCalibrator.calibrate();
                return null;
            }

            @Override
            protected void onPostExecute(Void result) {
                calibrationProgress.dismiss();
                mCalibrator.clearCorners();
                mOnCameraFrameRender = new OnCameraFrameRender(new CalibrationFrameRender(mCalibrator));
                String resultMessage = (mCalibrator.isCalibrated()) ?
                        res.getString(R.string.calibration_successful) + " " + mCalibrator.getAvgReprojectionError() :
                        res.getString(R.string.calibration_unsuccessful);
                (Toast.makeText(CameraCalibrationActivity.this, resultMessage, Toast.LENGTH_SHORT)).show();

                if (mCalibrator.isCalibrated()) {
                    CalibrationResult.save(CameraCalibrationActivity.this,
                            mCalibrator.getCameraMatrix(), mCalibrator.getDistortionCoefficients());
                }
            }
        }.execute();
    }

    /*@Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
        case R.id.calibration:
            mOnCameraFrameRender =
                new OnCameraFrameRender(new CalibrationFrameRender(mCalibrator));
            item.setChecked(true);
            return true;
        case R.id.undistortion:
            mOnCameraFrameRender =
                new OnCameraFrameRender(new UndistortionFrameRender(mCalibrator));
            item.setChecked(true);
            return true;
        case R.id.comparison:
            mOnCameraFrameRender =
                new OnCameraFrameRender(new ComparisonFrameRender(mCalibrator, mWidth, mHeight, getResources()));
            item.setChecked(true);
            return true;
        case R.id.calibrate:
            final Resources res = getResources();
            if (mCalibrator.getCornersBufferSize() < 2) {
                (Toast.makeText(this, res.getString(R.string.more_samples), Toast.LENGTH_SHORT)).show();
                return true;
            }

            mOnCameraFrameRender = new OnCameraFrameRender(new PreviewFrameRender());
            new AsyncTask<Void, Void, Void>() {
                private ProgressDialog calibrationProgress;

                @Override
                protected void onPreExecute() {
                    calibrationProgress = new ProgressDialog(CameraCalibrationActivity.this);
                    calibrationProgress.setTitle(res.getString(R.string.calibrating));
                    calibrationProgress.setMessage(res.getString(R.string.please_wait));
                    calibrationProgress.setCancelable(false);
                    calibrationProgress.setIndeterminate(true);
                    calibrationProgress.show();
                }

                @Override
                protected Void doInBackground(Void... arg0) {
                    mCalibrator.calibrate();
                    return null;
                }

                @Override
                protected void onPostExecute(Void result) {
                    calibrationProgress.dismiss();
                    mCalibrator.clearCorners();
                    mOnCameraFrameRender = new OnCameraFrameRender(new CalibrationFrameRender(mCalibrator));
                    String resultMessage = (mCalibrator.isCalibrated()) ?
                            res.getString(R.string.calibration_successful)  + " " + mCalibrator.getAvgReprojectionError() :
                            res.getString(R.string.calibration_unsuccessful);
                    (Toast.makeText(CameraCalibrationActivity.this, resultMessage, Toast.LENGTH_SHORT)).show();

                    if (mCalibrator.isCalibrated()) {
                        CalibrationResult.save(CameraCalibrationActivity.this,
                                mCalibrator.getCameraMatrix(), mCalibrator.getDistortionCoefficients());
                    }
                }
            }.execute();
            return true;
        default:
            return super.onOptionsItemSelected(item);
        }
    }*/

    public void onCameraViewStarted(int width, int height) {
        if (mWidth != width || mHeight != height) {
            mWidth = width;
            mHeight = height;
            mCalibrator = new CameraCalibrator(mWidth, mHeight);
            if (CalibrationResult.tryLoad(this, mCalibrator.getCameraMatrix(), mCalibrator.getDistortionCoefficients())) {
                mCalibrator.setCalibrated();
            }

            mOnCameraFrameRender = new OnCameraFrameRender(new CalibrationFrameRender(mCalibrator));
        }
    }

    public void onCameraViewStopped() {
    }

    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        return mOnCameraFrameRender.render(inputFrame);
    }

    @Override
    public boolean onTouch(View v, MotionEvent event) {
        Log.d(TAG, "onTouch invoked");

        mCalibrator.addCorners();
        return false;
    }
}

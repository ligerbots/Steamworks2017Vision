package erik.android.vision.visiontest;

import android.content.Context;
import android.content.res.Configuration;
import android.graphics.Matrix;
import android.graphics.Point;
import android.graphics.RectF;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.support.annotation.NonNull;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.util.Range;
import android.util.Size;
import android.util.SparseIntArray;
import android.view.Surface;
import android.view.TextureView;
import android.view.WindowManager;
import android.widget.Toast;

import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import erik.android.vision.visiontest_native.AppNative;

/**
 * The main activity class
 */
public class Camera2Activity extends AppCompatActivity {
    static {
        // "debug" isn't really debug. This means load OpenCV using the local copy instead of
        // going through OpenCV Manager. OpenCV Manager is annoying because it's async
        OpenCVLoader.initDebug();
    }

    private static final SparseIntArray ORIENTATIONS = new SparseIntArray();

    static {
        ORIENTATIONS.append(Surface.ROTATION_0, 90);
        ORIENTATIONS.append(Surface.ROTATION_90, 0);
        ORIENTATIONS.append(Surface.ROTATION_180, 270);
        ORIENTATIONS.append(Surface.ROTATION_270, 180);
    }

    private static final String TAG = "Camera2Activity";

    private String mCameraId;
    private AutoFitTextureView mTextureView;
    private CameraCaptureSession mCaptureSession;
    private CameraDevice mCameraDevice;
    private Size mPreviewSize;

    private HandlerThread mBackgroundThread;
    private Handler mBackgroundHandler;

    private ImageReader mImageReader;

    private FpsCounter mImageReaderFps = new FpsCounter("ImageReader");

    // dimensions of the image in NV21 and RGB format
    private int mNv21Width, mNv21Height, mRgbWidth, mRgbHeight;

    // helper classes
    private Calibration mCalibration;
    private ImageProcessor mImageProcessor;

    private CaptureRequest.Builder mPreviewRequestBuilder;
    private CaptureRequest mPreviewRequest;
    private final Semaphore mCameraOpenCloseLock = new Semaphore(1);
    private int mSensorOrientation;

    // set to true when camera parameters are updated and we need to restart the capture session
    private boolean mNeedsToRestartRepeatingCapture = false;

    private CrashRestarter crashRestarter;

    /**
     * Listener for when the display is ready. Opens the camera and configures displaying.
     * It's not really that important; I'm pretty sure the camera will still work without a visible
     * display but it's helpful to have and doesn't seem to impact performance
     */
    private final TextureView.SurfaceTextureListener mSurfaceTextureListener
            = new TextureView.SurfaceTextureListener() {

        @Override
        public void onSurfaceTextureAvailable(SurfaceTexture texture, int width, int height) {
            openCamera(width, height);
        }

        @Override
        public void onSurfaceTextureSizeChanged(SurfaceTexture texture, int width, int height) {
            configureTransform(width, height);
        }

        @Override
        public boolean onSurfaceTextureDestroyed(SurfaceTexture texture) {
            return true;
        }

        @Override
        public void onSurfaceTextureUpdated(SurfaceTexture texture) {
        }

    };

    /**
     * Because we want to know what's going on with the camera
     */
    private final CameraDevice.StateCallback mStateCallback = new CameraDevice.StateCallback() {

        @Override
        public void onOpened(@NonNull CameraDevice cameraDevice) {
            // This method is called when the camera is opened.  We start camera preview here.
            mCameraOpenCloseLock.release();
            mCameraDevice = cameraDevice;
            createCameraPreviewSession();
        }

        @Override
        public void onDisconnected(@NonNull CameraDevice cameraDevice) {
            mCameraOpenCloseLock.release();
            cameraDevice.close();
            mCameraDevice = null;
        }

        @Override
        public void onError(@NonNull CameraDevice cameraDevice, int error) {
            mCameraOpenCloseLock.release();
            cameraDevice.close();
            mCameraDevice = null;
            Camera2Activity.this.finish();
        }
    };


    /**
     * Recieves camera images and sends them to the image processor
     */
    private final ImageReader.OnImageAvailableListener mOnImageAvailableListener
            = new ImageReader.OnImageAvailableListener() {

        @Override
        public void onImageAvailable(ImageReader reader) {
            Image img = reader.acquireLatestImage();
            if (img == null) return;
            Log.i(TAG, "Image available");

            // Use native hacks to get a Mat fast
            // works on Nexus 5; is most likely phone-specific
            long nv21MatPtr = AppNative.copyNV21BufferToMat(
                    img.getPlanes()[0].getBuffer(), mNv21Width, mNv21Height);

            // now we need an RGB image
            Mat nv21Mat = new Mat(nv21MatPtr);
            Mat imageBgrMat = new Mat();
            Imgproc.cvtColor(nv21Mat, imageBgrMat, Imgproc.COLOR_YUV2BGR_NV12, 3);
            Mat imageRgbMat = new Mat();
            Imgproc.cvtColor(imageBgrMat, imageRgbMat, Imgproc.COLOR_BGR2RGB);
            Mat imageGrayMat = nv21Mat.submat(0, mRgbHeight, 0, mRgbWidth);

            // draw over the image using processing data
            mImageProcessor.postUpdateAndDraw(imageBgrMat, imageRgbMat);

            // send the image to SmartDashboard at a lower resolution
            Imgproc.pyrDown(imageRgbMat, imageRgbMat);
            Communications.cameraServerSendImage(imageRgbMat);
            Communications.sendBatteryLevel(Camera2Activity.this);

            nv21Mat.release();
            img.close();

            mImageReaderFps.feed();
        }
    };

    /**
     * Not really necessary, but the Google example used this
     */
    private CameraCaptureSession.CaptureCallback mCaptureCallback
            = new CameraCaptureSession.CaptureCallback() {

        private void process(CaptureResult result) {
            Log.i(TAG, "capture callback: " + result.toString());
        }

        @Override
        public void onCaptureProgressed(@NonNull CameraCaptureSession session,
                                        @NonNull CaptureRequest request,
                                        @NonNull CaptureResult partialResult) {
            process(partialResult);
        }

        @Override
        public void onCaptureCompleted(@NonNull CameraCaptureSession session,
                                       @NonNull CaptureRequest request,
                                       @NonNull TotalCaptureResult result) {
            process(result);
        }
    };

    private void showToast(final String text) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(Camera2Activity.this, text, Toast.LENGTH_SHORT).show();
            }
        });
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera2);
        // automatically unlock the phone, keep screen on, etc
        getWindow().addFlags(
                WindowManager.LayoutParams.FLAG_SHOW_WHEN_LOCKED |
                WindowManager.LayoutParams.FLAG_DISMISS_KEYGUARD |
                WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON |
                WindowManager.LayoutParams.FLAG_TURN_SCREEN_ON |
                WindowManager.LayoutParams.FLAG_ALLOW_LOCK_WHILE_SCREEN_ON);
        mTextureView = (AutoFitTextureView) findViewById(R.id.textureView);

        // get rid of those pesky "App has stopped" dialogs so we can restart on crashes
        Thread.UncaughtExceptionHandler handler = new Thread.UncaughtExceptionHandler() {
            @Override
            public void uncaughtException(Thread t, Throwable e) {
                e.printStackTrace();
                System.exit(-1);
            }
        };
        Thread.setDefaultUncaughtExceptionHandler(handler);
        Thread.currentThread().setUncaughtExceptionHandler(handler);

        crashRestarter = new CrashRestarter(this);

        // init communications
        Communications.enableUsbTethering(this);
        Communications.initNetworkTables();
        Communications.initCameraServer();

        new NTCommand("Vision/Crash", "Crash", new Runnable() {
            @Override
            public void run() {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        throw new RuntimeException("Triggered crash");
                    }
                });
            }
        });

        Communications.root.putBoolean("enabled", false);
        Communications.root.addTableListener("enabled", new ITableListener() {
            @Override
            public void valueChanged(ITable source, String key, final Object value, boolean isNew) {
                Camera2Activity.this.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        if(value.equals(Boolean.TRUE)) {
                            enableProcessing();
                        } else {
                            enableLowPowerMode();
                        }
                    }
                });
            }
        }, true);

        Log.i(TAG, "Phone info " + Build.MANUFACTURER + " " + Build.MODEL);

        if(!Build.MANUFACTURER.equals("LGE") || !Build.MODEL.equals("Nexus 5")) {
            // the issue is assuming that YUV_420_888 is equivalent to NV21. On the Nexus 5 it is;
            // however the NV21 format itself is not a supported capture format. Thus AppNative
            // has a hack to set a Mat data pointer to the Y channel ByteBuffer data pointer
            // of the YUV_420_888 image. This may need to be replace with a proper generic
            // YUV_420_888 to NV21 converter on other phones
            // eg http://stackoverflow.com/a/35221548/1021196
            Log.e(TAG, "Phone-specific native hacks may or may not work on this phone. " +
                    "Update the code and then remove this notice.");
            finish();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Communications.closeNetworkTables();
        Communications.closeCameraServer();
    }

    @Override
    public void onResume() {
        super.onResume();
        if(Communications.root.getBoolean("enabled", false)) {
            enableProcessing();
        }
    }

    @Override
    public void onPause() {
        enableLowPowerMode();
        super.onPause();
    }

    public void enableLowPowerMode() {
        closeCamera();
        stopBackgroundThread();
    }

    public void enableProcessing() {
        startBackgroundThread();

        // When the screen is turned off and turned back on, the SurfaceTexture is already
        // available, and "onSurfaceTextureAvailable" will not be called. In that case, we can open
        // a camera and start preview from here (otherwise, we wait until the surface is ready in
        // the SurfaceTextureListener).
        if (mTextureView.isAvailable()) {
            openCamera(mTextureView.getWidth(), mTextureView.getHeight());
        } else {
            mTextureView.setSurfaceTextureListener(mSurfaceTextureListener);
        }
    }

    private void setUpCameraOutputs(int width, int height) {
        CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
        try {
            for (String cameraId : manager.getCameraIdList()) {
                CameraCharacteristics characteristics
                        = manager.getCameraCharacteristics(cameraId);

                // We don't use a front facing camera
                Integer facing = characteristics.get(CameraCharacteristics.LENS_FACING);
                if (facing != null && facing == CameraCharacteristics.LENS_FACING_FRONT) {
                    continue;
                }

                StreamConfigurationMap map = characteristics.get(
                        CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                if (map == null) {
                    continue;
                }

                List<Size> captureSizes =
                        Arrays.asList(map.getOutputSizes(Parameters.CAPTURE_FORMAT));
                for (Size s : captureSizes) {
                    Log.i(TAG, "Supported capture size: " + s.toString());
                }
                Size captureSize = Parameters.CAPTURE_SIZE;
                Log.i(TAG, "Capturing at: "
                        + captureSize.getWidth() + "x" + captureSize.getHeight());
                mImageReader = ImageReader.newInstance(captureSize.getWidth(),
                        captureSize.getHeight(), Parameters.CAPTURE_FORMAT, 2);
                mNv21Width = captureSize.getHeight() + (captureSize.getHeight() / 2);
                mNv21Height = captureSize.getWidth();
                mRgbWidth = captureSize.getWidth();
                mRgbHeight = captureSize.getHeight();
                mImageReader.setOnImageAvailableListener(
                        mOnImageAvailableListener, mBackgroundHandler);

                // initialize calibration and processing
                if(mCalibration == null) {
                    mCalibration = new Calibration(mRgbWidth, mRgbHeight);
                    mImageProcessor = new ImageProcessor(mCalibration);
                }

                // find camera info and send it to SmartDashboard
                Range<Integer> camSensitivityRange =
                        characteristics.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE);
                Range<Long> camExposureTimeRange =
                        characteristics.get(CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE);
                Parameters.initCameraParameters(camSensitivityRange, camExposureTimeRange);
                Log.i(TAG, "Camera Info sensitivity range: "
                        + (Parameters.camSensitivityRange != null
                        ? Parameters.camSensitivityRange.toString() : null));
                Log.i(TAG, "Camera Info exposure time range: "
                        + (Parameters.camExposureTimeRange != null
                        ? Parameters.camExposureTimeRange.toString() : null));

                Long maxFrameDuration =
                        characteristics.get(CameraCharacteristics.SENSOR_INFO_MAX_FRAME_DURATION);
                long minFrameDuration =
                        map.getOutputMinFrameDuration(Parameters.CAPTURE_FORMAT, captureSize);
                Range<Long> frameTimeRange = new Range<>(minFrameDuration, maxFrameDuration);
                Log.i(TAG, "Camera Info frame time range: " + frameTimeRange.toString());
                float[] apertures =
                        characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_APERTURES);
                Log.i(TAG, "Camera Info available apertures: " + Arrays.toString(apertures));
                Float minFocus =
                        characteristics.get(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE);
                Log.i(TAG, "Camera Info focus range: 0.0-" + minFocus);

                // now the rest of this method is all Google example code

                // Find out if we need to swap dimension to get the preview size relative to sensor
                // coordinate.
                int displayRotation = getWindowManager().getDefaultDisplay().getRotation();
                //noinspection ConstantConditions
                mSensorOrientation = characteristics.get(CameraCharacteristics.SENSOR_ORIENTATION);
                boolean swappedDimensions = false;
                switch (displayRotation) {
                    case Surface.ROTATION_0:
                    case Surface.ROTATION_180:
                        if (mSensorOrientation == 90 || mSensorOrientation == 270) {
                            swappedDimensions = true;
                        }
                        break;
                    case Surface.ROTATION_90:
                    case Surface.ROTATION_270:
                        if (mSensorOrientation == 0 || mSensorOrientation == 180) {
                            swappedDimensions = true;
                        }
                        break;
                    default:
                        Log.e(TAG, "Display rotation is invalid: " + displayRotation);
                }

                Point displaySize = new Point();
                getWindowManager().getDefaultDisplay().getSize(displaySize);
                int rotatedPreviewWidth = width;
                int rotatedPreviewHeight = height;
                int maxPreviewWidth = displaySize.x;
                int maxPreviewHeight = displaySize.y;

                if (swappedDimensions) {
                    //noinspection SuspiciousNameCombination
                    rotatedPreviewWidth = height;
                    //noinspection SuspiciousNameCombination
                    rotatedPreviewHeight = width;
                    //noinspection SuspiciousNameCombination
                    maxPreviewWidth = displaySize.y;
                    //noinspection SuspiciousNameCombination
                    maxPreviewHeight = displaySize.x;
                }

                if (maxPreviewWidth > Parameters.MAX_PREVIEW_WIDTH) {
                    maxPreviewWidth = Parameters.MAX_PREVIEW_WIDTH;
                }

                if (maxPreviewHeight > Parameters.MAX_PREVIEW_HEIGHT) {
                    maxPreviewHeight = Parameters.MAX_PREVIEW_HEIGHT;
                }

                // Danger, W.R.! Attempting to use too large a preview size could  exceed the camera
                // bus' bandwidth limitation, resulting in gorgeous previews but the storage of
                // garbage capture data.
                mPreviewSize = chooseOptimalSize(map.getOutputSizes(SurfaceTexture.class),
                        rotatedPreviewWidth, rotatedPreviewHeight, maxPreviewWidth,
                        maxPreviewHeight, captureSize);

                // We fit the aspect ratio of TextureView to the size of preview we picked.
                int orientation = getResources().getConfiguration().orientation;
                if (orientation == Configuration.ORIENTATION_LANDSCAPE) {
                    mTextureView.setAspectRatio(
                            mPreviewSize.getWidth(), mPreviewSize.getHeight());
                } else {
                    mTextureView.setAspectRatio(
                            mPreviewSize.getHeight(), mPreviewSize.getWidth());
                }

                mCameraId = cameraId;
                return;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void openCamera(int width, int height) {
        setUpCameraOutputs(width, height);
        configureTransform(width, height);
        CameraManager manager = (CameraManager) this.getSystemService(Context.CAMERA_SERVICE);
        //noinspection TryWithIdenticalCatches
        try {
            if (!mCameraOpenCloseLock.tryAcquire(2500, TimeUnit.MILLISECONDS)) {
                throw new RuntimeException("Time out waiting to lock camera opening.");
            }
            manager.openCamera(mCameraId, mStateCallback, mBackgroundHandler);
        } catch (SecurityException e) {
            // permissions problem. Go enable the permission. I'm too lazy
            // to put in the code to ask for it
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void closeCamera() {
        try {
            mCameraOpenCloseLock.acquire();
            if (null != mCaptureSession) {
                mCaptureSession.close();
                mCaptureSession = null;
            }
            if (null != mCameraDevice) {
                mCameraDevice.close();
                mCameraDevice = null;
            }
            if (null != mImageReader) {
                mImageReader.close();
                mImageReader = null;
            }
        } catch (InterruptedException e) {
            throw new RuntimeException("Interrupted while trying to lock camera closing.", e);
        } finally {
            mCameraOpenCloseLock.release();
        }
    }

    private void startBackgroundThread() {
        mBackgroundThread = new HandlerThread("CameraBackground");
        mBackgroundThread.start();
        mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
    }

    private void stopBackgroundThread() {
        if(mBackgroundThread == null) return;
        mBackgroundThread.quitSafely();
        try {
            mBackgroundThread.join();
            mBackgroundThread = null;
            mBackgroundHandler = null;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * We try to start the app right on boot. Sometimes the camera isn't ready and throws random
     * exceptions. This method handles errors by restarting the camera session
     */
    private void restartCameraSession() {
        //
        Log.i(TAG, "Restarting camera session");
        try {
            closeCamera();
        } catch(Exception e) {
            // don't care
            e.printStackTrace();
        }
        if (mTextureView.isAvailable()) {
            this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    openCamera(mTextureView.getWidth(), mTextureView.getHeight());
                }
            });
        } else {
            mTextureView.setSurfaceTextureListener(mSurfaceTextureListener);
        }
    }

    private void createCameraPreviewSession() {
        try {
            SurfaceTexture texture = mTextureView.getSurfaceTexture();
            if(texture == null) {
                restartCameraSession();
                return;
            }

            // We configure the size of default buffer to be the size of camera preview we want.
            texture.setDefaultBufferSize(mPreviewSize.getWidth(), mPreviewSize.getHeight());

            // This is the output Surface we need to start preview.
            Surface surface = new Surface(texture);

            // We set up a CaptureRequest.Builder with the output Surface.
            mPreviewRequestBuilder
                    = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
            mPreviewRequestBuilder.addTarget(surface);
            mPreviewRequestBuilder.addTarget(mImageReader.getSurface());

            // Here, we create a CameraCaptureSession for camera preview.
            mCameraDevice.createCaptureSession(Arrays.asList(surface, mImageReader.getSurface()),
                    new CameraCaptureSession.StateCallback() {

                        @Override
                        public void onConfigured(
                                @NonNull CameraCaptureSession cameraCaptureSession) {
                            // The camera is already closed
                            if (null == mCameraDevice) {
                                return;
                            }

                            // When the session is ready, we start displaying the preview.
                            mCaptureSession = cameraCaptureSession;

                            try {
                                // Auto focus should be continuous for camera preview.
                                mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AF_MODE,
                                        CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);

                                mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AE_MODE,
                                        CaptureRequest.CONTROL_AE_MODE_OFF);
                                mPreviewRequestBuilder.set(CaptureRequest.SENSOR_EXPOSURE_TIME,
                                        (long) Parameters.exposureTime.getValue());
                                mPreviewRequestBuilder.set(CaptureRequest.SENSOR_SENSITIVITY,
                                        (int) Parameters.sensitivity.getValue());

                                // Finally, we start displaying the camera preview.
                                mPreviewRequest = mPreviewRequestBuilder.build();
                                try {
                                    mCaptureSession.setRepeatingRequest(mPreviewRequest,
                                            mCaptureCallback, mBackgroundHandler);
                                } catch(IllegalStateException | SecurityException e) {
                                    // session closed, restart it
                                    restartCameraSession();
                                    return;
                                }
                            } catch (CameraAccessException e) {
                                e.printStackTrace();
                            }

                            // when parameters are updated via SmartDashboard, restart the capture
                            // session
                            Parameters.setCameraParametersUpdateRunnable(new Runnable(){
                                @Override
                                public void run() {
                                    Camera2Activity.this.runOnUiThread(new Runnable() {
                                        @Override
                                        public void run() {
                                            Log.i(TAG, "NT restart triggered");
                                            try {
                                                mPreviewRequestBuilder.set(
                                                        CaptureRequest.SENSOR_EXPOSURE_TIME,
                                                        (long) Parameters.exposureTime.getValue());
                                                mPreviewRequestBuilder.set(
                                                        CaptureRequest.SENSOR_SENSITIVITY,
                                                        (int) Parameters.sensitivity.getValue());
                                                mPreviewRequest = mPreviewRequestBuilder.build();
                                                mNeedsToRestartRepeatingCapture = true;
                                                try {
                                                    mCaptureSession.stopRepeating();
                                                } catch(IllegalStateException |
                                                        SecurityException e) {
                                                    // session closed
                                                    restartCameraSession();
                                                }
                                            } catch (Exception e) {
                                                Log.e(TAG, "Camera error", e);
                                            }
                                        }
                                    });
                                }
                            });
                        }

                        @Override
                        public void onConfigureFailed(
                                @NonNull CameraCaptureSession cameraCaptureSession) {
                            showToast("Failed");
                        }

                        @Override
                        public void onReady (@NonNull CameraCaptureSession session) {
                            if(mNeedsToRestartRepeatingCapture) {
                                Log.i(TAG, "NT restart session");
                                mNeedsToRestartRepeatingCapture = false;
                                try {
                                    mCaptureSession.setRepeatingRequest(mPreviewRequest,
                                            mCaptureCallback, mBackgroundHandler);
                                } catch (CameraAccessException e) {
                                    e.printStackTrace();
                                } catch(IllegalStateException | SecurityException e) {
                                    // session closed
                                    restartCameraSession();
                                }
                            }
                        }
                    }, null
            );
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    // rest of this file is Google example code
    private void configureTransform(int viewWidth, int viewHeight) {
        if (null == mTextureView || null == mPreviewSize) {
            return;
        }
        int rotation = getWindowManager().getDefaultDisplay().getRotation();
        Matrix matrix = new Matrix();
        RectF viewRect = new RectF(0, 0, viewWidth, viewHeight);
        RectF bufferRect = new RectF(0, 0, mPreviewSize.getHeight(), mPreviewSize.getWidth());
        float centerX = viewRect.centerX();
        float centerY = viewRect.centerY();
        if (Surface.ROTATION_90 == rotation || Surface.ROTATION_270 == rotation) {
            bufferRect.offset(centerX - bufferRect.centerX(), centerY - bufferRect.centerY());
            matrix.setRectToRect(viewRect, bufferRect, Matrix.ScaleToFit.FILL);
            float scale = Math.max(
                    (float) viewHeight / mPreviewSize.getHeight(),
                    (float) viewWidth / mPreviewSize.getWidth());
            matrix.postScale(scale, scale, centerX, centerY);
            matrix.postRotate(90 * (rotation - 2), centerX, centerY);
        } else if (Surface.ROTATION_180 == rotation) {
            matrix.postRotate(180, centerX, centerY);
        }
        mTextureView.setTransform(matrix);
    }

    @SuppressWarnings("unused")
    private int getOrientation(int rotation) {
        // Sensor orientation is 90 for most devices, or 270 for some devices (eg. Nexus 5X)
        // We have to take that into account and rotate the image properly.
        // For devices with orientation of 90, we simply return our mapping from ORIENTATIONS.
        // For devices with orientation of 270, we need to rotate the image 180 degrees.
        return (ORIENTATIONS.get(rotation) + mSensorOrientation + 270) % 360;
    }

    private static Size chooseOptimalSize(Size[] choices, int textureViewWidth,
                                          int textureViewHeight, int maxWidth, int maxHeight,
                                          Size aspectRatio) {

        // Collect the supported resolutions that are at least as big as the preview Surface
        List<Size> bigEnough = new ArrayList<>();
        // Collect the supported resolutions that are smaller than the preview Surface
        List<Size> notBigEnough = new ArrayList<>();
        int w = aspectRatio.getWidth();
        int h = aspectRatio.getHeight();
        for (Size option : choices) {
            if (option.getWidth() <= maxWidth && option.getHeight() <= maxHeight &&
                    option.getHeight() == option.getWidth() * h / w) {
                if (option.getWidth() >= textureViewWidth &&
                        option.getHeight() >= textureViewHeight) {
                    bigEnough.add(option);
                } else {
                    notBigEnough.add(option);
                }
            }
        }

        // Pick the smallest of those big enough. If there is no one big enough, pick the
        // largest of those not big enough.
        if (bigEnough.size() > 0) {
            return Collections.min(bigEnough, new CompareSizesByArea());
        } else if (notBigEnough.size() > 0) {
            return Collections.max(notBigEnough, new CompareSizesByArea());
        } else {
            Log.e(TAG, "Couldn't find any suitable preview size");
            return choices[0];
        }
    }

    static class CompareSizesByArea implements Comparator<Size> {

        @Override
        public int compare(Size lhs, Size rhs) {
            // We cast here to ensure the multiplications won't overflow
            return Long.signum((long) lhs.getWidth() * lhs.getHeight() -
                    (long) rhs.getWidth() * rhs.getHeight());
        }

    }
}

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

import erik.android.vision.visiontest_native.AppNative;

public class Camera2Activity extends AppCompatActivity {
    static {
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

    private int mNv21Width, mNv21Height, mRgbWidth, mRgbHeight;

    private Calibration mCalibration;
    private ImageProcessor mImageProcessor;

    private CaptureRequest.Builder mPreviewRequestBuilder;
    private CaptureRequest mPreviewRequest;
    private final Semaphore mCameraOpenCloseLock = new Semaphore(1);
    private int mSensorOrientation;

    private boolean mNeedsToRestartRepeatingCapture = false;

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

            Mat nv21Mat = new Mat(nv21MatPtr);
            Mat imageBgrMat = new Mat();
            Imgproc.cvtColor(nv21Mat, imageBgrMat, Imgproc.COLOR_YUV2BGR_NV12, 3);
            Mat imageRgbMat = new Mat();
            Imgproc.cvtColor(imageBgrMat, imageRgbMat, Imgproc.COLOR_BGR2RGB);

            mImageProcessor.postUpdateAndDraw(imageBgrMat, imageRgbMat);

            if(mCalibration.isEnabled()) {
                mCalibration.processFrame(nv21Mat.submat(0, mRgbHeight, 0, mRgbWidth), imageRgbMat);
            }

            //Imgproc.pyrDown(imageRgbMat, imageRgbMat);
            //Communications.cameraServerSendImage(imageRgbMat);
            Mat toEncode = new Mat();
            Imgproc.cvtColor(imageRgbMat, toEncode, Imgproc.COLOR_BGR2YUV_I420);
            RTPStreaming.encodeFrame(toEncode);
            toEncode.release();
            imageBgrMat.release();
            imageRgbMat.release();
            nv21Mat.release();
            img.close();

            mImageReaderFps.feed();
        }
    };

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
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        mTextureView = (AutoFitTextureView) findViewById(R.id.textureView);

        Communications.initNetworkTables();
        //Communications.initCameraServer();
        AppNative.ffmpegInit();
        RTPStreaming.init("Erik-PC", 5809);

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

    @Override
    public void onPause() {
        closeCamera();
        stopBackgroundThread();
        super.onPause();
    }

    private void setUpCameraOutputs(int width, int height) {
        CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
        try {
            for (String cameraId : manager.getCameraIdList()) {
                CameraCharacteristics characteristics
                        = manager.getCameraCharacteristics(cameraId);

                // We don't use a front facing camera in this sample.
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
                Size largest = Parameters.CAPTURE_SIZE;
                Log.i(TAG, "Capturing at: " + largest.getWidth() + "x" + largest.getHeight());
                mImageReader = ImageReader.newInstance(largest.getWidth(), largest.getHeight(),
                        Parameters.CAPTURE_FORMAT, 2);
                mNv21Width = largest.getHeight() + (largest.getHeight() / 2);
                mNv21Height = largest.getWidth();
                mRgbWidth = largest.getWidth();
                mRgbHeight = largest.getHeight();
                mImageReader.setOnImageAvailableListener(
                        mOnImageAvailableListener, mBackgroundHandler);

                if(mCalibration == null) {
                    mCalibration = new Calibration(mRgbWidth, mRgbHeight);
                    mImageProcessor = new ImageProcessor(mCalibration);
                }

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
                        map.getOutputMinFrameDuration(Parameters.CAPTURE_FORMAT, largest);
                Range<Long> frameTimeRange = new Range<>(minFrameDuration, maxFrameDuration);
                Log.i(TAG, "Camera Info frame time range: " + frameTimeRange.toString());
                float[] apertures =
                        characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_APERTURES);
                Log.i(TAG, "Camera Info available apertures: " + Arrays.toString(apertures));
                Float minFocus =
                        characteristics.get(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE);
                Log.i(TAG, "Camera Info focus range: 0.0-" + minFocus);

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
                        maxPreviewHeight, largest);

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
        mBackgroundThread.quitSafely();
        try {
            mBackgroundThread.join();
            mBackgroundThread = null;
            mBackgroundHandler = null;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void createCameraPreviewSession() {
        try {
            SurfaceTexture texture = mTextureView.getSurfaceTexture();
            assert texture != null;

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
                                mCaptureSession.setRepeatingRequest(mPreviewRequest,
                                        mCaptureCallback, mBackgroundHandler);
                            } catch (CameraAccessException e) {
                                e.printStackTrace();
                            }

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
                                                mCaptureSession.stopRepeating();
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
                                }
                            }
                        }
                    }, null
            );
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

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

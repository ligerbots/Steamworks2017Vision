package erik.android.vision.visiontest;


import android.graphics.ImageFormat;
import android.util.Range;
import android.util.Size;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

/**
 * Stores constants and runtime parameters
 */
public class Parameters {
    public static final double DEFAULT_CALIB_DOT_SPACING = 4.15625; // in
    public static final double DEFAULT_TARGET_WIDTH = 12.0; // in
    public static final double DEFAULT_TARGET_HEIGHT = 8.0; // in

    public static final int MAX_PREVIEW_WIDTH = 1920;
    public static final int MAX_PREVIEW_HEIGHT = 1080;

    public static final int CAPTURE_FORMAT = ImageFormat.YUV_420_888;
    public static final Size CAPTURE_SIZE = new Size(640, 480);

    public static Range<Integer> camSensitivityRange;
    public static Range<Long> camExposureTimeRange;

    public static NTBoundedNumber sensitivity;
    public static NTBoundedNumber exposureTime;

    private static ITable targetSizeTable;

    private static Runnable cameraParametersUpdateRunnable;
    private static ITableListener refresh = new ITableListener() {
        @Override
        public void valueChanged(ITable source, String key, Object value, boolean isNew) {
            if(cameraParametersUpdateRunnable != null) {
                cameraParametersUpdateRunnable.run();
            }
        }
    };

    public static void initDefaultVariables() {
        ITable calibTable = NetworkTable.getTable(Calibration.TABLE_NAME);
        if(!calibTable.containsKey("squareSize"))
            calibTable.putNumber("squareSize", DEFAULT_CALIB_DOT_SPACING);

        targetSizeTable = NetworkTable.getTable("Vision/target");
        if(!targetSizeTable.containsKey("width"))
            targetSizeTable.putNumber("width", DEFAULT_TARGET_WIDTH);
        if(!targetSizeTable.containsKey("height"))
            targetSizeTable.putNumber("height", DEFAULT_TARGET_HEIGHT);
    }

    public static double[] getTargetSize() {
        return new double[]{
                targetSizeTable.getNumber("width", DEFAULT_TARGET_WIDTH),
                targetSizeTable.getNumber("height", DEFAULT_TARGET_HEIGHT)
        };
    }

    public static void initCameraParameters(Range<Integer> camSensitivityRange,
                                            Range<Long> camExposureTimeRange) {
        Parameters.camSensitivityRange = camSensitivityRange;
        Parameters.camExposureTimeRange = camExposureTimeRange;

        if(sensitivity != null) return;

        sensitivity = new NTBoundedNumber("Vision/sensitivity", camSensitivityRange.getLower(),
                camSensitivityRange.getUpper());
        exposureTime = new NTBoundedNumber("Vision/exposureTime", camExposureTimeRange.getLower(),
                camExposureTimeRange.getUpper());
    }

    public static void setCameraParametersUpdateRunnable(Runnable run) {
        cameraParametersUpdateRunnable = run;
        sensitivity.getTable().addTableListener(refresh, true);
        exposureTime.getTable().addTableListener(refresh, true);
    }
}

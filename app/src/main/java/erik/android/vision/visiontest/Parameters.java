package erik.android.vision.visiontest;


import android.content.Context;
import android.graphics.ImageFormat;
import android.os.Environment;
import android.util.Log;
import android.util.Range;
import android.util.Size;

import org.json.JSONObject;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

/**
 * Stores constants and runtime parameters
 */
public class Parameters {
    private static final String TAG = "Parameters";

    private static String SAVE_FILE;
    private static String PURPOSE_FILE;

    public enum Purpose {
        GEAR_LIFT("Vision_Gear", (byte) 0x93),
        BOILER("Vision_Boiler", (byte) 0xB0);

        String visionTable;
        byte dataCode;
        Purpose(String visionTable, byte dataCode) {
            this.visionTable = visionTable;
            this.dataCode = dataCode;
        }
    }

    public static Purpose purpose;

    public static final double DEFAULT_CALIB_DOT_SPACING = 4.15625; // in

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
            JSONObject obj = new JSONObject();
            try {
                obj.put("sensitivity", sensitivity.getValue());
                obj.put("exposureTime", exposureTime.getValue());
                FileWriter out = new FileWriter(SAVE_FILE);
                out.write(obj.toString());
                out.close();
            } catch (Exception e) {
                e.printStackTrace();
            }


            if(cameraParametersUpdateRunnable != null) {
                cameraParametersUpdateRunnable.run();
            }
        }
    };

    public static void initPurpose(Context ctx) {
        SAVE_FILE = Environment.getExternalStorageDirectory().getPath() + "/camerasettings.json";
        PURPOSE_FILE = Environment.getExternalStorageDirectory().getPath() + "/purpose.txt";
        StringBuilder builder = new StringBuilder();
        try {
            FileReader reader = new FileReader(PURPOSE_FILE);
            char[] c = new char[1024];
            int b;
            while ((b = reader.read(c)) > -1) {
                builder.append(c, 0, b);
            }
        } catch (Exception e) {
            Log.e(TAG, "Error", e);
        }

        String thisInstancePurpose = builder.toString();

        if (thisInstancePurpose.contains("boiler")) {
            purpose = Purpose.BOILER;
        } else {
            purpose = Purpose.GEAR_LIFT;
        }
    }

    public static void initDefaultVariables() {
        ITable calibTable = NetworkTable.getTable(purpose.visionTable + "/Calibration");
        if(!calibTable.containsKey("squareSize"))
            calibTable.putNumber("squareSize", DEFAULT_CALIB_DOT_SPACING);

        // target width and height handled by Robot
    }

    public static double[] getTargetSize() {
        return new double[]{
                targetSizeTable.getNumber("width", Double.NaN),
                targetSizeTable.getNumber("height", Double.NaN)
        };
    }

    public static void initCameraParameters(Range<Integer> camSensitivityRange,
                                            Range<Long> camExposureTimeRange) {
        Parameters.camSensitivityRange = camSensitivityRange;
        Parameters.camExposureTimeRange = camExposureTimeRange;

        if(sensitivity != null) return;

        sensitivity = new NTBoundedNumber(purpose.visionTable + "/sensitivity", camSensitivityRange.getLower(),
                camSensitivityRange.getUpper());
        exposureTime = new NTBoundedNumber(purpose.visionTable + "/exposureTime", camExposureTimeRange.getLower(),
                camExposureTimeRange.getUpper());

        if(new File(SAVE_FILE).exists()) {
            try {
                FileReader in = new FileReader(SAVE_FILE);
                StringBuilder b = new StringBuilder();
                char[] buf = new char[1024];
                int len;
                while((len = in.read(buf)) != -1) {
                    b.append(buf, 0, len);
                }
                JSONObject obj = new JSONObject(b.toString());
                sensitivity.setValue(obj.getDouble("sensitivity"));
                exposureTime.setValue(obj.getDouble("exposureTime"));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public static void setCameraParametersUpdateRunnable(Runnable run) {
        cameraParametersUpdateRunnable = run;
        sensitivity.getTable().addTableListener(refresh, true);
        exposureTime.getTable().addTableListener(refresh, true);
    }
}

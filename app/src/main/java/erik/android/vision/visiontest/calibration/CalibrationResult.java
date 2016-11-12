package erik.android.vision.visiontest.calibration;

import android.app.Activity;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.Log;

import org.json.JSONArray;
import org.json.JSONObject;
import org.opencv.core.Mat;

public abstract class CalibrationResult {
    private static final String TAG = "CalibrationResult";

    private static final int CAMERA_MATRIX_ROWS = 3;
    private static final int CAMERA_MATRIX_COLS = 3;
    private static final int DISTORTION_COEFFICIENTS_SIZE = 5;

    protected static SharedPreferences sharedPref;

    protected static void initPrefs(Activity activity){
        if(sharedPref == null){
            sharedPref = PreferenceManager.getDefaultSharedPreferences(activity.getApplicationContext());
        }
    }

    public static void save(Activity activity, Mat cameraMatrix, Mat distortionCoefficients) {
        initPrefs(activity);
        SharedPreferences.Editor editor = sharedPref.edit();

        double[] cameraMatrixArray = new double[CAMERA_MATRIX_ROWS * CAMERA_MATRIX_COLS];
        cameraMatrix.get(0,  0, cameraMatrixArray);

        double[] distortionCoefficientsArray = new double[DISTORTION_COEFFICIENTS_SIZE];
        distortionCoefficients.get(0, 0, distortionCoefficientsArray);
        try {
            JSONObject root = new JSONObject();
            JSONArray cam = new JSONArray(cameraMatrixArray);
            JSONArray dist = new JSONArray(distortionCoefficientsArray);
            root.put("cameraMatrix", cam);
            root.put("distortionCoefficients", dist);
            editor.putString("calibration", root.toString());
        } catch(Exception e){
            e.printStackTrace();
        }

        editor.commit();
        Log.i(TAG, "Saved camera matrix: " + cameraMatrix.dump());
        Log.i(TAG, "Saved distortion coefficients: " + distortionCoefficients.dump());
    }

    public static boolean tryLoad(Activity activity, Mat cameraMatrix, Mat distortionCoefficients) {
        initPrefs(activity);
        String rootString = sharedPref.getString("calibration", "");
        if (rootString.length() == 0) {
            Log.i(TAG, "No previous calibration results found");
            return false;
        }

        Log.i(TAG, "Params raw JSON string:");
        Log.i(TAG, rootString);

        try {
            JSONObject root = new JSONObject(rootString);

            JSONArray cam = root.getJSONArray("cameraMatrix");
            for(int i = 0; i < CAMERA_MATRIX_ROWS; i++){
                for(int j = 0; j < CAMERA_MATRIX_COLS; j++) {
                    cameraMatrix.put(i, j, cam.getDouble(i * CAMERA_MATRIX_ROWS + j));
                }
            }
            Log.i(TAG, "Loaded camera matrix: " + cameraMatrix.dump());

            JSONArray dist = root.getJSONArray("distortionCoefficients");
            for(int i = 0; i < dist.length(); i++){
                distortionCoefficients.put(i, 0, dist.getDouble(i));
            }
            Log.i(TAG, "Loaded distortion coefficients: " + distortionCoefficients.dump());

            return true;
        } catch(Exception e) {
            e.printStackTrace();
            return false;
        }
    }
}

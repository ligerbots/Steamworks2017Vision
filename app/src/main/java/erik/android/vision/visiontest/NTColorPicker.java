package erik.android.vision.visiontest;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class NTColorPicker {
    public enum ColorMode {
        HSV(new float[][]{
                new float[]{0, 180},
                new float[]{0, 255},
                new float[]{0, 255}
        });

        float[][] ranges;
        ColorMode(float[][] ranges) {
            this.ranges = ranges;
        }
    }

    private NetworkTable mTable;
    private ColorMode mMode;

    public NTColorPicker(String path, ColorMode mode) {
        mTable = NetworkTable.getTable(path);
        mTable.putString("colorMode", mode.toString());
        mTable.putBoolean("enabled", false);
        this.mMode = mode;
    }

    public Scalar getLower() {
        double[] value = mTable.getNumberArray("lower", new double[]{0, 0, 0});
        float ranges[][] = mMode.ranges;
        for(int i = 0; i < ranges.length; i++) {
            value[i] = value[i] * ranges[i][1] / 255d;
        }
        Log.i("NTColorPicker", "lower " + Arrays.toString(value));
        return new Scalar(value);
    }

    public Scalar getUpper() {
        double[] value = mTable.getNumberArray("upper", new double[]{0, 0, 0});
        float ranges[][] = mMode.ranges;
        for(int i = 0; i < ranges.length; i++) {
            value[i] = value[i] * ranges[i][1] / 255d;
        }
        Log.i("NTColorPicker", "upper " + Arrays.toString(value));
        return new Scalar(value);
    }

    public void setHistogram(Mat image) {
        if(!mTable.getBoolean("enabled", false)) {
            return;
        }
        List<Mat> planes = new ArrayList<Mat>();
        Core.split(image, planes);

        MatOfInt histSize = new MatOfInt(256);

        float ranges[][] = mMode.ranges;

        boolean accumulate = false;

        for(int i = 0; i < planes.size(); i++) {
            Mat hist = new Mat();
            MatOfFloat histRange = new MatOfFloat(ranges[i]);
            Imgproc.calcHist(Collections.singletonList(planes.get(i)), new MatOfInt(0), new Mat(), hist, histSize, histRange, accumulate);
            float[] histIntArray = new float[hist.rows() * hist.cols()];
            hist.get(0, 0, histIntArray);
            double[] histDoubleArray = new double[histIntArray.length];
            for(int j = 0; j < histIntArray.length; j++) {
                histDoubleArray[j] = histIntArray[j];
            }
            mTable.putNumberArray("hist_" + i, histDoubleArray);
        }
    }
}

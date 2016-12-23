package erik.android.vision.visiontest;


import android.util.Log;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * Counts FPS
 */
public class FpsCounter {
    private static final String TAG = "FpsCounter";

    private double mFrameCount = 0;
    private long mLastTime;
    private double mFps = 0;
    private String mName;

    public FpsCounter(String name) {
        mLastTime = System.currentTimeMillis();
        this.mName = name;
    }

    public void feed() {
        mFrameCount++;

        if (System.currentTimeMillis() - mLastTime > 1000) {
            double dtMs = (double) (System.currentTimeMillis() - mLastTime);
            double dtS = dtMs / 1000;
            mFps = (mFrameCount / dtS);
            Log.i(TAG, mName + ": " + mFps);
            mLastTime = System.currentTimeMillis();
            mFrameCount = 0;
            NetworkTable.getTable("Vision").putNumber("fps_" + mName, mFps);
        }
    }

    public double getFps() {
        return mFps;
    }
}

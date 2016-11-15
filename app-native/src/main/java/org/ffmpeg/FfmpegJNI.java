package org.ffmpeg;

import erik.android.vision.visiontest_native.AppNative;

public class FfmpegJNI {
    static {
        AppNative.load();
    }
    public static native String helloWorldJNI();
}

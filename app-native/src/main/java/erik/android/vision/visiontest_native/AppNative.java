package erik.android.vision.visiontest_native;

public class AppNative {
    static {
        System.loadLibrary("app-native");
    }

    public static native String helloWorldJni();
}

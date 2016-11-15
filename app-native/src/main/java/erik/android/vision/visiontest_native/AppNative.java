package erik.android.vision.visiontest_native;

public class AppNative {
    static {
        load();
    }

    public static void load() {
        System.loadLibrary("app-native");
    }
}

package erik.android.vision.visiontest_native;

import java.nio.ByteBuffer;

public class AppNative {
    static {
        // these all compile as versioned files (.so.12.34.56) but Android Studio ignores them
        // unless they end in .so. Because of this, Java is unable to resolve the dependencies by
        // itself so we have to help it out by carefully loading all the libs in the correct order
        System.loadLibrary("x264");
        System.loadLibrary("avutil"); // x264
        System.loadLibrary("swresample"); // x264, avutil
        System.loadLibrary("avcodec"); // x264, swresample, avutil
        System.loadLibrary("avformat"); // x264, avcodec, avutil
        // now all the dependencies are prepared, so the main native code should load
        System.loadLibrary("app-native");
    }

    public static native long copyNV21BufferToMat(ByteBuffer buffer, int width, int height);

    public static native void copyMatOfByteToCameraServerPacket(ByteBuffer packet, long matAddress);

    public static native void ffmpegInit();
    public static native boolean streamingInit(String ip, int port);
    public static native boolean streamingEncodeFrame(long matAddress);
    public static native boolean streamingSend();
}

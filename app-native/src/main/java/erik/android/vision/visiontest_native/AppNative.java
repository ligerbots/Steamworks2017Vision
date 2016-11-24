package erik.android.vision.visiontest_native;

import java.nio.ByteBuffer;

public class AppNative {
    static {
        System.loadLibrary("app-native");
    }

    public static native long copyNV21BufferToMat(ByteBuffer buffer, int width, int height);

    public static native void copyMatOfByteToCameraServerPacket(ByteBuffer packet, long matAddress);
}

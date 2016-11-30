package erik.android.vision.visiontest;

import android.util.Log;

import org.opencv.core.Mat;

import erik.android.vision.visiontest_native.AppNative;

public class RTPStreaming {
    private static final String TAG = "RTPStreaming";

    private static Thread senderThread;
    private static Runnable sender = new Runnable() {
        @Override
        public void run() {
            long s = System.currentTimeMillis();
            int n = 0;
            while(true) {
                // send buffered packets until we run out or there's an error
                while(AppNative.streamingSend()) {
                    n++;
                    // don't use *all* of the CPU maybe
                    Thread.yield();
                }

                if(System.currentTimeMillis() - s > 5000) {
                    Log.i(TAG, "Encoding fps: " + (n/((System.currentTimeMillis() - s)/1000)));
                    s = System.currentTimeMillis();
                    n = 0;
                }

                Thread.yield();
            }
        }
    };

    public static void init(String ip, int port) {
        boolean success = AppNative.streamingInit(ip, port);
        if(success) {
            senderThread = new Thread(sender);
            senderThread.setName("RTP Streaming Sender");
            senderThread.setDaemon(true);
            senderThread.start();
        } else {
            Log.e(TAG, "Streaming init failed");
        }
    }

    public static void encodeFrame(Mat i420) {
        boolean result = AppNative.streamingEncodeFrame(i420.nativeObj);
        if(!result) {
            Log.e(TAG, "Frame encode failed");
        }
    }
}

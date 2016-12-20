package erik.android.vision.visiontest;

import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfInt;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.Enumeration;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.networktables.NetworkTablesJNI;
import erik.android.vision.visiontest_native.AppNative;

/**
 * Exactly what it sounds like. Communications
 */
public class Communications {
    static final String TAG = "Communications";

    public static final String PERSISTENT_FILENAME = "/storage/emulated/0/networktables.ini";
    public static final String IDENTITY = "Android";
    public static final int CS_CONTROL_PORT = 5809;
    public static final int CS_STREAM_PORT = 5810;
    public static final byte[] CS_MAGIC_NUMBER = new byte[]{1, 0, 0, 0};
    public static final int DATA_PORT = 5808;

    protected static InetAddress broadcastAddress = null;
    protected static DatagramChannel udpCameraServerChannel = null;
    protected static MatOfByte sendBuffer = new MatOfByte();
    protected static long lastSendTime = 0;

    public static void initNetworkTables() {
        NetworkTable.setClientMode();
        NetworkTable.setNetworkIdentity(IDENTITY);
        NetworkTable.setIPAddress("Erik-PC"); // TODO: change to roboRIO!
        NetworkTable.setPersistentFilename(PERSISTENT_FILENAME);
        NetworkTable.initialize();
    }

    public static void closeNetworkTables() {
        NetworkTable.shutdown();
    }

    public static boolean isNetworkTablesConnected() {
        return NetworkTablesJNI.getConnections().length > 0;
    }

    public static void initCameraServer() {
        try {
            findBroadcastAddress();

            if(udpCameraServerChannel != null) {
                return;
            }

            udpCameraServerChannel = DatagramChannel.open();
            udpCameraServerChannel.socket().setBroadcast(true);
            udpCameraServerChannel.socket().setReuseAddress(true);
            udpCameraServerChannel.socket().bind(new InetSocketAddress(CS_CONTROL_PORT));
            udpCameraServerChannel.configureBlocking(true);
        } catch(Exception e) {
            Log.e(TAG, "UDP error", e);
        }
    }

    public static void closeCameraServer() {
        try {
            udpCameraServerChannel.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        udpCameraServerChannel = null;
    }

    public static void dataServerSendData(double rvec_0, double rvec_1, double rvec_2,
                                          double tvec_0, double tvec_1, double tvec_2) {
        ByteBuffer packet = ByteBuffer.allocateDirect(Double.SIZE / 8 * 6);
        packet.position(0);
        packet.putDouble(rvec_0);
        packet.putDouble(rvec_1);
        packet.putDouble(rvec_2);
        packet.putDouble(tvec_0);
        packet.putDouble(tvec_1);
        packet.putDouble(tvec_2);
        try {
            udpCameraServerChannel.send(packet, new InetSocketAddress(broadcastAddress, DATA_PORT));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void cameraServerSendImage(Mat rgbFrame) {
        /*if(System.currentTimeMillis() - lastSendTime < 1000/CS_FPS) {
            return;
        }*/
        lastSendTime = System.currentTimeMillis();
        if(broadcastAddress == null) {
            findBroadcastAddress();
            if(broadcastAddress == null) {
                return;
            }
        }

        Log.i(TAG, "Image size: "
                + rgbFrame.size() + " / type: " + rgbFrame.channels() + ":" + rgbFrame.depth());
        Imgcodecs.imencode(".jpg", rgbFrame, sendBuffer,
                new MatOfInt(Imgcodecs.CV_IMWRITE_JPEG_QUALITY, 80));
        int dataLength = sendBuffer.rows() * sendBuffer.cols();
        ByteBuffer packet = ByteBuffer.allocateDirect(dataLength + 8);
        packet.position(0);
        packet.put(CS_MAGIC_NUMBER);
        packet.putInt(dataLength);
        packet.position(0);
        // yay more native performance hacks
        AppNative.copyMatOfByteToCameraServerPacket(packet, sendBuffer.getNativeObjAddr());
        try {
            udpCameraServerChannel.send(packet,
                    new InetSocketAddress(broadcastAddress, CS_STREAM_PORT));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Finds the USB networking interface (rndis0) and gets the broadcast address
     */
    protected static void findBroadcastAddress() {
        if(broadcastAddress != null) return;
        try {
            Enumeration<NetworkInterface> allInterfaces = NetworkInterface.getNetworkInterfaces();
            while (allInterfaces.hasMoreElements()) {
                NetworkInterface networkInterface = allInterfaces.nextElement();
                if(networkInterface.getName().equals("rndis0") && networkInterface.isUp()) {
                    for(InterfaceAddress addr : networkInterface.getInterfaceAddresses()) {
                        if(addr == null) continue;
                        InetAddress bcast = addr.getBroadcast();
                        if(bcast == null) continue;
                        Log.i(TAG, "Broadcast address: " + bcast.toString());
                        broadcastAddress = bcast;
                    }
                }
            }
        } catch(Exception e) {
            Log.e(TAG, "Network error", e);
        }
    }

    /**
     * Uses root to automatically enable USB tethering. This allows the entire startup sequence to
     * be completely automatic (as soon as the robot is powered, the phone boots, the app starts,
     * and it enables tethering by itself) which reduces the potential for field setup mistakes
     */
    public static void enableUsbTethering(final Context context) {
        Thread usbSetupThread = new Thread(new Runnable() {
            @Override
            public void run() {
                // keep polling until USB is up
                while(true) {
                    if (isUsbConnected(context)) {
                        // http://stackoverflow.com/a/24346101/1021196
                        try {
                            Log.i(TAG, "Enabling tethering");
                            Runtime.getRuntime().exec("su -c service call connectivity 30 i32 1");
                            return;
                        } catch(Exception e) {
                            Log.e(TAG, "Error enabling tethering", e);
                        }
                    }
                    try {
                        Thread.sleep(1000);
                    } catch(Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        });
        usbSetupThread.setName("USB Tethering Setup Thread");
        usbSetupThread.setDaemon(true);
        usbSetupThread.start();
    }

    public static boolean isUsbConnected(Context context) {
        Intent intent = context.registerReceiver(null,
                new IntentFilter("android.hardware.usb.action.USB_STATE"));
        return intent.getExtras().getBoolean("connected");
    }
}

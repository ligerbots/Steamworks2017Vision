package erik.android.vision.visiontest;

import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.BatteryManager;
import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfInt;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.Enumeration;

import edu.wpi.first.wpilibj.networktables.ConnectionInfo;
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
    public static final String ROBORIO_ADDRESS = "roborio-2877-frc";

    protected static InetAddress roboRioAddress = null;
    protected static InetSocketAddress roboRioDataAddress = null;
    protected static InetSocketAddress roboRioCameraStreamAddress = null;
    protected static DatagramChannel udpCameraServerChannel = null;
    protected static MatOfByte sendBuffer = new MatOfByte();
    protected static long lastSendTime = 0;
    protected static long lastHeardFromRio = 0;

    protected static NetworkTable root;

    public static void initNetworkTables() {
        NetworkTable.setClientMode();
        NetworkTable.setNetworkIdentity(IDENTITY);
        NetworkTable.setIPAddress(ROBORIO_ADDRESS);
        NetworkTable.setPersistentFilename(PERSISTENT_FILENAME);
        NetworkTable.initialize();
        Parameters.initDefaultVariables();

        root = NetworkTable.getTable("Vision");
    }

    public static void closeNetworkTables() {
        NetworkTable.shutdown();
    }

    public static boolean isNetworkTablesConnected() {
        return NetworkTablesJNI.getConnections().length > 0;
    }

    public static void initCameraServer() {
        try {
            findRoboRioAddress();

            if(udpCameraServerChannel != null) {
                return;
            }

            udpCameraServerChannel = DatagramChannel.open();
            //udpCameraServerChannel.socket().setBroadcast(true);
            udpCameraServerChannel.socket().setReuseAddress(true);
            udpCameraServerChannel.socket().bind(new InetSocketAddress(CS_CONTROL_PORT));
            udpCameraServerChannel.configureBlocking(true);

            Thread receiver = new Thread(new Runnable() {
                @Override
                public void run() {
                    cameraServerReceiverThread();
                }
            });
            receiver.setDaemon(true);
            receiver.setName("Camera Server Feedback Thread");
            receiver.start();
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
        checkRoboRioAddress();
        if(roboRioDataAddress == null) {
            return;
        }

        ByteBuffer packet = ByteBuffer.allocateDirect(Double.SIZE / 8 * 6);
        packet.position(0);
        packet.putDouble(rvec_0);
        packet.putDouble(rvec_1);
        packet.putDouble(rvec_2);
        packet.putDouble(tvec_0);
        packet.putDouble(tvec_1);
        packet.putDouble(tvec_2);
        try {
            udpCameraServerChannel.send(packet, roboRioDataAddress);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    protected static void checkRoboRioAddress() {
        if(roboRioAddress == null || System.currentTimeMillis() - lastHeardFromRio > 2000) {
            findRoboRioAddress();
        }
    }

    public static void cameraServerSendImage(Mat rgbFrame) {
        /*if(System.currentTimeMillis() - lastSendTime < 1000/CS_FPS) {
            return;
        }*/
        lastSendTime = System.currentTimeMillis();

        checkRoboRioAddress();
        if(roboRioCameraStreamAddress == null) {
            return;
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
                    roboRioCameraStreamAddress);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void cameraServerReceiverThread() {
        ByteBuffer buf;
        try {
            buf = ByteBuffer.allocateDirect(udpCameraServerChannel.socket().getReceiveBufferSize());
        } catch(Exception e) {
            Log.e(TAG, "UDP error", e);
            return;
        }
        while(udpCameraServerChannel.isOpen()) {
            try {
                SocketAddress from = udpCameraServerChannel.receive(buf);
                if(from != null) {
                    lastHeardFromRio = System.currentTimeMillis();
                }
            } catch(Exception e) {
                Log.e(TAG, "UDP error", e);
            }
        }
    }

    protected static double getBatteryLevel(Context ctx) {
        Intent batteryIntent = ctx.registerReceiver(null, new IntentFilter(Intent.ACTION_BATTERY_CHANGED));
        int level = batteryIntent.getIntExtra(BatteryManager.EXTRA_LEVEL, -1);
        int scale = batteryIntent.getIntExtra(BatteryManager.EXTRA_SCALE, -1);

        // Error checking that probably isn't needed but I added just in case.
        if(level == -1 || scale == -1) {
            return -1;
        }

        return (level * 100.0) / scale;
    }

    protected static boolean isCharging(Context ctx) {
        IntentFilter ifilter = new IntentFilter(Intent.ACTION_BATTERY_CHANGED);
        Intent batteryStatus = ctx.registerReceiver(null, ifilter);
        int status = batteryStatus.getIntExtra(BatteryManager.EXTRA_STATUS, -1);
        boolean isCharging = status == BatteryManager.BATTERY_STATUS_CHARGING ||
                status == BatteryManager.BATTERY_STATUS_FULL;
        return isCharging;
    }

    public static void initBatteryMonitor(final Context ctx) {
        Thread batteryMonitorThread = new Thread(new Runnable() {
            @Override
            public void run() {
                long lastChargeTime = System.currentTimeMillis();
                while(true) {
                    root.putNumber("battery", getBatteryLevel(ctx));

                    if(isCharging(ctx)) {
                        lastChargeTime = System.currentTimeMillis();
                    } else {
                        if(System.currentTimeMillis() - lastChargeTime > 1000 * 60 * 2) {
                            try {
                                Log.i(TAG, "Shutting down");
                                Runtime.getRuntime().exec("su -c reboot -p");
                            } catch(Exception e) {
                                Log.e(TAG, "Error shutting down", e);
                            }
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
        batteryMonitorThread.setName("Battery Monitor Thread");
        batteryMonitorThread.setDaemon(true);
        batteryMonitorThread.start();
    }

    protected static void findRoboRioAddress() {
        try {
            ConnectionInfo[] connectionInfos = NetworkTablesJNI.getConnections();
            ConnectionInfo robotInfo = null;
            for(ConnectionInfo connectionInfo: connectionInfos) {
                if(connectionInfo.remote_id.equals("Robot")) {
                    robotInfo = connectionInfo;
                    break;
                }
            }
            if(robotInfo == null) return;

            roboRioAddress = InetAddress.getByName(robotInfo.remote_ip);
            roboRioCameraStreamAddress = new InetSocketAddress(roboRioAddress, CS_STREAM_PORT);
            roboRioDataAddress = new InetSocketAddress(roboRioAddress, DATA_PORT);
        } catch(Exception e) {
            Log.e(TAG, "Network error resolving roborio", e);
            roboRioAddress = null;
            roboRioCameraStreamAddress = null;
            roboRioDataAddress = null;
        }
    }

    protected static NetworkInterface findUsbTetheringInterface() {
        try {
            Enumeration<NetworkInterface> allInterfaces = NetworkInterface.getNetworkInterfaces();
            while (allInterfaces.hasMoreElements()) {
                NetworkInterface networkInterface = allInterfaces.nextElement();
                if(networkInterface.getName().equals("rndis0") && networkInterface.isUp()) {
                    return networkInterface;
                }
            }
        } catch(Exception e) {
            Log.e(TAG, "Network error finding rndis0", e);
        }
        return null;
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
                    if (isUsbConnected(context) && findUsbTetheringInterface() == null) {
                        try {
                            Thread.sleep(3000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        // http://stackoverflow.com/a/24346101/1021196
                        try {
                            Log.i(TAG, "Enabling tethering");
                            Runtime.getRuntime().exec("su -c service call connectivity 30 i32 1");
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

package erik.android.vision.visiontest;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class NTClient {
    public static final String PERSISTENT_FILENAME = "/storage/emulated/0/networktables.ini";
    public static final String IDENTITY = "Android";

    public static void initNetworkTables() {
        NetworkTable.setClientMode();
        NetworkTable.setNetworkIdentity(IDENTITY);
        NetworkTable.setIPAddress("Erik-PC");
        NetworkTable.setPersistentFilename(PERSISTENT_FILENAME);
        NetworkTable.initialize();
    }

    public static void closeNetworkTables() {
        NetworkTable.shutdown();
    }
}

package erik.android.vision.visiontest;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

public class NTCommand extends ITableListener implements Runnable {
    private Runnable mAction;
    private String mName;
    private ITable mTable;
    private Thread mActionThread;

    public NTCommand(String path, String name, Runnable action) {
        this.mAction = action;
        this.mName = name;
        mActionThread = null;
        mTable = NetworkTable.getTable(path);
        mTable.putString("name", name);
        mTable.putBoolean("running", false);
        mTable.putBoolean("isParented", false);

        mTable.addTableListener("running", this, true);
    }

    @Override
    public void valueChanged(ITable source, String key, Object value, boolean isNew) {
        boolean isRunning = (Boolean) value;
        if(isRunning && mActionThread == null) {
            mActionThread = new Thread(this);
            mActionThread.setName("NTCommand " + mName);
            mActionThread.start();
        }
    }

    @Override
    public void run() {
        mTable.putBoolean("running", true);
        mAction.run();
        mTable.putBoolean("running", false);
        mActionThread = null;
    }
}

package erik.android.vision.visiontest;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * A NetworkTables object that represents a number bounded by an upper and lower value
 */
public class NTBoundedNumber {
    public static final String LOWER = "lower";
    public static final String UPPER = "upper";
    public static final String VALUE = "value";

    private ITable mTable;

    public NTBoundedNumber(String path) {
        this.mTable = NetworkTable.getTable(path);
    }

    public NTBoundedNumber(String path, double lower, double upper) {
        this(path);

        setLower(lower);
        setUpper(upper);
    }

    public ITable getTable() {
        return mTable;
    }

    public double getLower() {
        return mTable.getNumber(LOWER, 0);
    }

    public double getUpper() {
        return mTable.getNumber(UPPER, 0);
    }

    public double getValue() {
        double value = mTable.getNumber(VALUE, 0);
        double lower = getLower();
        double upper = getUpper();
        if(value < lower) {
            value = lower;
        } else if(value > upper) {
            value = upper;
        }
        return value;
    }

    public void setLower(double lower) {
        mTable.putNumber(LOWER, lower);
    }

    public void setUpper(double upper) {
        mTable.putNumber(UPPER, upper);
    }

    public void setValue(double value) {
        mTable.putNumber(VALUE, value);
    }
}

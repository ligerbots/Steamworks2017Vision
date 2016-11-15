package erik.android.vision.visiontest;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

public class NTBoundedNumber {
    public static final String LOWER = "lower";
    public static final String UPPER = "upper";
    public static final String VALUE = "value";
    private ITable table;

    public NTBoundedNumber(String path) {
        this.table = NetworkTable.getTable(path);
    }

    public NTBoundedNumber(String path, double lower, double upper) {
        this(path);

        setLower(lower);
        setUpper(upper);
    }

    public ITable getTable() {
        return table;
    }

    public double getLower() {
        return table.getNumber(LOWER, 0);
    }

    public double getUpper() {
        return table.getNumber(UPPER, 0);
    }

    public double getValue() {
        double value = table.getNumber(VALUE, 0);
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
        table.putNumber(LOWER, lower);
    }

    public void setUpper(double upper) {
        table.putNumber(UPPER, upper);
    }

    public void setValue(double value) {
        table.putNumber(VALUE, value);
    }
}

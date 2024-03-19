package frc.robot.util.power.battery;

/** We use this in setting the state of the {@link Battery} and then for the {@link RobotPowerDistribution} to limit current on subsystem that might 
 * draw too much current. */
public enum BatteryStatus {
    
    FULL_CHARGE(14.0, 12.8),
    HALF_CHARGE(12.8, 12.4),
    LOW_CHARGE(12.4, 11.8),
    CRITICAL_CHARGE(11.8, 10.5);

    private final double UPPER_RANGE_VOLTAGE;
    private final double LOWER_RANGE_VOLTAGE;

    /**
     * Makes the range of voltages in which we set the status of the battery. This status is then used to determine configuration
     * for power-hungry systems.
     * @param upperRangeVoltage Top Voltage in Range
     * @param lowerRangeVoltage Bottom Voltage in Range
     */
    BatteryStatus(double upperRangeVoltage, double lowerRangeVoltage) {
        this.UPPER_RANGE_VOLTAGE = upperRangeVoltage;
        this.LOWER_RANGE_VOLTAGE = lowerRangeVoltage;
    }

    /**
     * @return The uppmost voltage in the range.
     */
    public double getUpperRangeVoltage() {
        return UPPER_RANGE_VOLTAGE;
    }

    /**
     * @return The lowest voltage in the range. 
     */
    public double getLowerRangeVoltage() {
        return LOWER_RANGE_VOLTAGE;
    }

    /**
     * Checks if the voltage is wihtin the specified range.
     * @param voltage Voltage of Battery in Resting State.
     * @return True if the voltage is within the range. 
     */
    public boolean isInRange(double voltage) {
        return voltage >= LOWER_RANGE_VOLTAGE && voltage <= UPPER_RANGE_VOLTAGE;
    }

    /**
     * Based on battery voltage we determine that status of the battery assuming resting conditions.
     * @param voltage Battery Voltage in Resting State.
     * @return Status of Battery.
     */
    public static BatteryStatus getStatus(double voltage) {
        if (FULL_CHARGE.isInRange(voltage)) {
            return FULL_CHARGE;
        } else if (HALF_CHARGE.isInRange(voltage)) {
            return HALF_CHARGE;
        } else if (LOW_CHARGE.isInRange(voltage)) {
            return LOW_CHARGE;
        } else {
            return CRITICAL_CHARGE;
        }
    }

    public String toString() {
        switch (this) {
            case FULL_CHARGE:
                return "Full Charge";
            case HALF_CHARGE:
                return "Half Charge";
            case LOW_CHARGE:
                return "Low Charge";
            case CRITICAL_CHARGE:
                return "Critically Low Charge";
            default:
                return "Unknown";
        }
    }
}

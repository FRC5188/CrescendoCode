package frc.robot.util.power.battery;

import edu.wpi.first.wpilibj.RobotController;

public abstract class Battery {
    private static double lowestRecordedVoltage = 14.0; // This isn't a possible value. Therefore if something goes wrong we know. 

    /**
     * @return The status of the battery based on the lowest recorded voltage assuring the worst-case we're worst off than we actually are.
     */
    public static BatteryStatus getStatus() {
        if (lowestRecordedVoltage >= BatteryStatus.FULL_CHARGE.getLowerRangeVoltage()){
            return BatteryStatus.FULL_CHARGE;
        }
        else if (lowestRecordedVoltage >= BatteryStatus.HALF_CHARGE.getLowerRangeVoltage()){
            return BatteryStatus.HALF_CHARGE;
        }
        else if (lowestRecordedVoltage >= BatteryStatus.LOW_CHARGE.getLowerRangeVoltage()){
            return BatteryStatus.LOW_CHARGE;
        }
        else {
            return BatteryStatus.CRITICAL_CHARGE;
        }
    }

    /**
     * Should be regulary called to update the lowest recorded voltage. In playing it safe we'll take lowest as it's better to have less
     * power and the assurance that we won't brown out than to have more power and risk browning out.
     */
    public static void updateVoltage() {
        if (RobotController.getBatteryVoltage() < lowestRecordedVoltage){
            lowestRecordedVoltage = RobotController.getBatteryVoltage();
        }
    }
}

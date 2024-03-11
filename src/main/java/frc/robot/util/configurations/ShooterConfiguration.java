package frc.robot.util.configurations;

import frc.robot.RobotConfiguration;

public abstract class ShooterConfiguration {
    public static boolean IS_USING_ANGLE_ADJUSTMENT = true;
    public static boolean IS_USING_FLYWHEELS = true;

    static {
        if (!RobotConfiguration.IS_USING_SHOOTER) {
            IS_USING_ANGLE_ADJUSTMENT = false;
            IS_USING_FLYWHEELS = false;
        }
    }
}

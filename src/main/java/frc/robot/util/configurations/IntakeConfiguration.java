package frc.robot.util.configurations;

import frc.robot.RobotConfiguration;

public abstract class IntakeConfiguration {
    public static boolean IS_USING_INTAKE_ROLLERS = true;
    public static boolean IS_USING_INTAKE_PIVOT = true;
    public static boolean IS_USING_INTAKE_FEEDER = true;

    // If we're not using the intake then we'll turn off all of the intake features.
    // Else we can individually turn off the features.
    static {
        if (!RobotConfiguration.IS_USING_INTAKE) {
            IS_USING_INTAKE_ROLLERS = false;
            IS_USING_INTAKE_PIVOT = false;
            IS_USING_INTAKE_FEEDER = false;
        }
    }
}

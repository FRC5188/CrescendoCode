package frc.robot.hardware;

public class HardwareConstants {
    public class CanIds {
        
        // |====================== INTAKE SUBSYSTEM CONSTANTS ======================|
        public static int PIVOT_MOTOR_ID = 0;
        public static int ROLLER_MOTOR_ID = 0;

        // |====================== SHOOTER SUBSYSTEM CONSTANTS ======================|
        public static int ANGLE_MOTOR_ID = 0;
        public static int TOP_FLYWHEEL_MOTOR_ID = 0;
        public static int BOTTOM_FLYWHEEL_MOTOR_ID = 0;

        //|====================== CLIMBER SUBSYSTEM CONSTANTS ======================|
        public static int LEFT_CLIMBER_MOTOR = 0;
        public static int RIGHT_CLIMBER_MOTOR = 0;
    }

    public class DIOPorts {
        public static int LIGHT_SENSOR_PORT = 0;
        public static int SHOOTER_ANGLE_ENCODER_PORT = 1; // These must be configured when robot is wired. 
    }
}

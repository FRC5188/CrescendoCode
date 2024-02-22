package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class HardwareConstants {
    public static final int NUMBER_OF_CAMERAS = 2;
    public class CanIds {
        
        // |====================== INTAKE SUBSYSTEM CAN IDs ======================|
        public static int PIVOT_MOTOR_ID = 21;
        public static int ROLLER_MOTOR_ID = 20;

        // |====================== SHOOTER SUBSYSTEM CAN IDs ======================|
        public static int ANGLE_MOTOR_ID = 30;
        public static int LEFT_FLYWHEEL_MOTOR_ID = 31;
        public static int RIGHT_FLYWHEEL_MOTOR_ID = 32;

        //|====================== CLIMBER SUBSYSTEM CAN IDs ======================|
        public static int LEFT_CLIMBER_MOTOR = 40;
        public static int RIGHT_CLIMBER_MOTOR = 41;

        //|====================== LED SUBSYSTEM CAN IDS ==========================|
        public static int CANDLE_ID = 50;
        
    }

    public class DIOPorts {
        public static int SHOOTER_ANGLE_ENCODER_PORT = 7; // These must be configured when robot is wired. 
        public static int INTAKE_PIVOT_ENCODER_PORT = 8;
    }

    public class ComponentTransformations {
    // |====================== START VISION SUBSYSTEM TRANSFORMATIONS ======================|
        /*
        * Assume that you're looking at the robot from above it. In our code we treat
        * the robot like a single point in an XY-Plane. Where the front of the robot is
        * the
        * positive X, and where the left side of the robot is the negative Y.
        */
        // How far foward/backward the camera is from robot center.
        private static final double CAMERA_ONE_X_FROM_ROBOT_CENTER = 0.1586; //Ch
        // How far left/right the camera is from robot center.
        private static final double CAMERA_ONE_Y_FROM_ROBOT_CENTER = 0.2794;
        // How far up/down the camera is from center if we look at robot from side in 3D
        // space.
        private static final double CAMERA_ONE_Z_FROM_ROBOT_CENTER = 0.1752;
    
        private static final double CAMERA_ONE_ROLL = 0;
        private static final double CAMERA_ONE_PITCH = Math.toRadians(20.0);
        private static final double CAMERA_ONE_YAW = 0;

        private static Transform3d _cameraOnePosition = new Transform3d(
        new Translation3d(CAMERA_ONE_X_FROM_ROBOT_CENTER, CAMERA_ONE_Y_FROM_ROBOT_CENTER, CAMERA_ONE_Z_FROM_ROBOT_CENTER),
        new Rotation3d(CAMERA_ONE_ROLL, CAMERA_ONE_PITCH, CAMERA_ONE_YAW));

        private static final double CAMERA_TWO_X_FROM_ROBOT_CENTER = 0.0334;
        private static final double CAMERA_TWO_Y_FROM_ROBOT_CENTER = -0.2794;
        private static final double CAMERA_TWO_Z_FROM_ROBOT_CENTER = 0.1752;

        private static final double CAMERA_TWO_ROLL = 0;
        private static final double CAMERA_TWO_PITCH = Math.toRadians(20.0);
        private static final double CAMERA_TWO_YAW = Math.toRadians(180.0);

        private static Transform3d _cameraTwoPosition = new Transform3d(
        new Translation3d(CAMERA_TWO_X_FROM_ROBOT_CENTER, CAMERA_TWO_Y_FROM_ROBOT_CENTER, CAMERA_TWO_Z_FROM_ROBOT_CENTER),
        new Rotation3d(CAMERA_TWO_ROLL, CAMERA_TWO_PITCH, CAMERA_TWO_YAW));

        public static Transform3d[] _cameraPosition = new Transform3d[] {_cameraOnePosition, _cameraTwoPosition};
    
    // |====================== END VISION SUBSYSTEM TRANSFORMATIONS ======================|
    
    }

    public class AbsEncoderOffsets {
        public static final double INTAKE_PIVOT_ENCODER_OFFSET_IN_DEGREES = 51.98;
        public static final double SHOOTER_ANGLE_ENCODER_OFFSET_IN_DEGREES = 228.64;
    }
}

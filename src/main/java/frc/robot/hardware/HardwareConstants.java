package frc.robot.hardware;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class HardwareConstants {
    public static final int NUMBER_OF_CAMERAS = 1;
    public class CanIds {
        
        // |====================== INTAKE SUBSYSTEM CAN IDs ======================|
        public static int PIVOT_MOTOR_ID = 0;
        public static int ROLLER_MOTOR_ID = 1;

        // |====================== SHOOTER SUBSYSTEM CAN IDs ======================|
        public static int ANGLE_MOTOR_ID = 2;
        public static int TOP_FLYWHEEL_MOTOR_ID = 3;
        public static int BOTTOM_FLYWHEEL_MOTOR_ID = 4;

        //|====================== CLIMBER SUBSYSTEM CAN IDs ======================|
        public static int LEFT_CLIMBER_MOTOR = 5;
        public static int RIGHT_CLIMBER_MOTOR = 6;

        //|====================== LED SUBSYSTEM CAN IDS ==========================|
        public static int CANDLE_ID = 7;
        
    }

    public class DIOPorts {
        public static int SHOOTER_ANGLE_ENCODER_PORT = 0; // These must be configured when robot is wired. 
        public static int INTAKE_ANGLE_ENCODER_PORT = 1;
    }
    public class ComponentTransformations {

    // |====================== START VISION SUBSYSTEM TRANSFORMATIONS ======================|

        public static final String CAMERA_ONE_NAME = "photoncamera";
        /*
        * Assume that you're looking at the robot from above it. In our code we treat
        * the robot like a single point in an XY-Plane. Where the front of the robot is
        * the
        * positive X, and where the left side of the robot is the negative Y.
        */
        // How far foward/backward the camera is from robot center.
        private static final double CAMERA_ONE_X_FROM_ROBOT_CENTER = 0.193;
        // How far left/right the camera is from robot center.
        private static final double CAMERA_ONE_Y_FROM_ROBOT_CENTER = 0.2794;
        // How far up/down the camera is from center if we look at robot from side in 3D
        // space.
        private static final double CAMERA_ONE_Z_FROM_ROBOT_CENTER = 0.375;
    
        private static final double CAMERA_ONE_ROLL = 0;
        private static final double CAMERA_ONE_PITCH = 0;
        private static final double CAMERA_ONE_YAW = Math.toRadians(-10.5);

        private static Transform3d _cameraOnePosition = new Transform3d(
        new Translation3d(CAMERA_ONE_X_FROM_ROBOT_CENTER, CAMERA_ONE_Y_FROM_ROBOT_CENTER, CAMERA_ONE_Z_FROM_ROBOT_CENTER),
        new Rotation3d(CAMERA_ONE_ROLL, CAMERA_ONE_PITCH, CAMERA_ONE_YAW));

        public static final String CAMERA_TWO_NAME = "photoncamera";
        private static final double CAMERA_TWO_X_FROM_ROBOT_CENTER = 0;
        private static final double CAMERA_TWO_Y_FROM_ROBOT_CENTER = 0;
        private static final double CAMERA_TWO_Z_FROM_ROBOT_CENTER = 0;

        private static final double CAMERA_TWO_ROLL = 0;
        private static final double CAMERA_TWO_PITCH = 0;
        private static final double CAMERA_TWO_YAW = Math.toRadians(0);

        private static Transform3d _cameraTwoPosition = new Transform3d(
        new Translation3d(CAMERA_TWO_X_FROM_ROBOT_CENTER, CAMERA_TWO_Y_FROM_ROBOT_CENTER, CAMERA_TWO_Z_FROM_ROBOT_CENTER),
        new Rotation3d(CAMERA_TWO_ROLL, CAMERA_TWO_PITCH, CAMERA_TWO_YAW));

        public static Transform3d[] _cameraPosition = new Transform3d[] {_cameraOnePosition, _cameraTwoPosition};
    
    // |====================== END VISION SUBSYSTEM TRANSFORMATIONS ======================|
    
    }
}

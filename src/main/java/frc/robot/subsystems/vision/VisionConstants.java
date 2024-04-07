package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * <STRONG>Robot Orientation for Vision (Same As WPI Defaults):</STRONG> <p></p>
 * - X Axis: Toward (Positive) & Into (Negative) Front of Robot. <p></p>
 * - Y Axis: Left (Negative) & Right (Positive) Front of Robot. <p></p>
 * - Z Axis: Up (Positive) & Down (Negative) Front of Robot. <p></p>
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">WPI Documentation</a> uses
 * the same convenstions for robot coordinate systems.
 */
public abstract class VisionConstants {

    /*
     * Physical Transformations of the Cameras on Robot.
     */
    public static abstract class Physical {
        /**<STRONG> Calibration Tutorial (PhotonVision): </STORNG> 
         * - Calibration is the most important step of this. Bad calibration data will lead to bad estimations that are diffecult to initially diagnose since
         *   they could also be mechanical transformation issues with the camera. For reference use @see <a href="https://docs.photonvision.org/en/latest/docs/calibration/calibration.html">PhotonVision Documentation</a> 
         *   though you can either use the on-board calibration or @see <a href="https://calibdb.net/">this calibration tool</a>. In our experience better results will come from calibration with the online tool. If you're
         *   using on-board calibration make sure that you use more than the minimum number of pictures and that you have good coverage of the corners of the camera.
         * - Then you can add the mechanical transformation to the cameras. These are the offsets from the geometric center of the robot to the camera. For more information on the coordinate 
         *   plane see the documentation for this class. When doing this sit down with a mechanical student and make sure you both understand the coordinate place.
         * - The constants you get are likley to be somewhat wrong due to installation issues. The simple way we can compansate for this is by measuring the distance of our robot from a known field
         *   element and then adding the difference to our constants. In 2023 we drove the robot so odometry thought it was touching the stage, then measured its actua distance 
         *   from the stage. This won't be perfect but an inch or two is more than in the margin of error for our vision.
         * - Also if you're reading this I hope you have a great season. Vision isn't an easy, and sometimes quite frustrating subystem to work on but it's also so rewarding when everything works.
         *   Thank you for taking up the sword and getting vision working!^^ 
        */

        // Please Document Whenever Cameras Were Last Calibrated. 
        private static final double CAMERA_ONE_X_FROM_ROBOT_CENTER = -0.032 + 0.279; // Calibration (State Competition | April 7)
        private static final double CAMERA_ONE_Y_FROM_ROBOT_CENTER = 0.273;          // Calibration (State Competition | April 7)
        private static final double CAMERA_ONE_Z_FROM_ROBOT_CENTER = 0.2583;         // Calibration (State Competition | April 7)

        private static final double CAMERA_ONE_ROLL = 0;
        private static final double CAMERA_ONE_PITCH = Math.toRadians(20.0);
        private static final double CAMERA_ONE_YAW = Math.toRadians(0.0);

        private static final double CAMERA_TWO_X_FROM_ROBOT_CENTER = -0.16 - 0.254; // Calibration (State Competition | April 7)
        private static final double CAMERA_TWO_Y_FROM_ROBOT_CENTER = -0.273;         // Calibration (State Competition | April 7)
        private static final double CAMERA_TWO_Z_FROM_ROBOT_CENTER = 0.2583;         // Calibration (State Competition | April 7)

        private static final double CAMERA_TWO_ROLL = 0;
        private static final double CAMERA_TWO_PITCH = Math.toRadians(20.0);
        private static final double CAMERA_TWO_YAW = Math.toRadians(180.0);

        private static final double CAMERA_THREE_X_FROM_ROBOT = 0.2508 - 0.46355;    // Calibration (State Competition | April 7)
        private static final double CAMERA_THREE_Y_FROM_ROBOT = -0.2508 + 0.74295 + 0.34925 - 0.0365; // Calibration (State Competition | April 7)
        private static final double CAMERA_THREE_Z_FROM_ROBOT = 0.27;                // Calibration (State Competition | April 7)

        private static final double CAMERA_THREE_ROLL = 0.0;
        private static final double CAMERA_THREE_PITCH = Math.toRadians(28.125);
        private static final double CAMERA_THREE_YAW = Math.toRadians(90.0);

        private static final double CAMERA_FOUR_X_FROM_ROBOT = 0.2508; //- 0.40645;  // Calibration (State Competition | April 7)
        private static final double CAMERA_FOUR_Y_FROM_ROBOT = -0.2508 - 0.41275 - 0.1905; // Calibration (State Competition | April 7)
        private static final double CAMERA_FOUR_Z_FROM_ROBOT = 0.27;                // Calibration (State Competition | April 7)

        private static final double CAMERA_FOUR_ROLL = 0.0;
        private static final double CAMERA_FOUR_PITCH = Math.toRadians(28.125);
        private static final double CAMERA_FOUR_YAW = Math.toRadians(270.0);

        public static final Transform3d _cameraOnePosition = new Transform3d(
            new Translation3d(CAMERA_ONE_X_FROM_ROBOT_CENTER, CAMERA_ONE_Y_FROM_ROBOT_CENTER, CAMERA_ONE_Z_FROM_ROBOT_CENTER),
            new Rotation3d(CAMERA_ONE_ROLL, CAMERA_ONE_PITCH, CAMERA_ONE_YAW)
        );

        public static final Transform3d _cameraTwoPosition = new Transform3d(
            new Translation3d(CAMERA_TWO_X_FROM_ROBOT_CENTER, CAMERA_TWO_Y_FROM_ROBOT_CENTER, CAMERA_TWO_Z_FROM_ROBOT_CENTER),
            new Rotation3d(CAMERA_TWO_ROLL, CAMERA_TWO_PITCH, CAMERA_TWO_YAW)
        );

        public static final Transform3d _cameraThreePosition = new Transform3d(
            new Translation3d(CAMERA_THREE_X_FROM_ROBOT, CAMERA_THREE_Y_FROM_ROBOT, CAMERA_THREE_Z_FROM_ROBOT),
            new Rotation3d(CAMERA_THREE_ROLL, CAMERA_THREE_PITCH, CAMERA_THREE_YAW)
        );

        public static final Transform3d _cameraFourPosition = new Transform3d(
            new Translation3d(CAMERA_FOUR_X_FROM_ROBOT, CAMERA_FOUR_Y_FROM_ROBOT, CAMERA_FOUR_Z_FROM_ROBOT),
            new Rotation3d(CAMERA_FOUR_ROLL, CAMERA_FOUR_PITCH, CAMERA_FOUR_YAW)
        );

        public static final Transform3d[] _cameraPosition = new Transform3d[] {_cameraOnePosition, _cameraTwoPosition, _cameraThreePosition, _cameraFourPosition};
    }

    /*
     * Software Constants Such as Number of Cameras Used & For Filtering Tags.
     */
    public static abstract class Software {
        public static final int NUMBER_OF_CAMERAS = 4;

        public static final double AMBIGUITY_CUTOFF = 0.2;
    }
}
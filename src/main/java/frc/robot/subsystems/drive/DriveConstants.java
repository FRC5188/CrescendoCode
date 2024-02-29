package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    // Swerve constants
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(17.6);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(20.75);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(20.75);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    // Speaker Positions in inches
    // See https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf for diagram
    public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), new Rotation2d(0));
    public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));
    
    /**
     * 
     * timestep used in {@link #ChassisSpeeds.discretize(CassisSpeeds, double)}
     *  */ 
    public static final double DISCRETE_TIMESTEP = 0.02;

    /**
     * 
     * SWERVE MODULE ENCODER OFFSETS
     * 
     * If tuning these follow these steps:
     *  -set constants to zero in this file
     *  - deploy code
     *  - use something flat and make all modules face exactly forward.
     *      - the bevel gear needs to face to the right on BOTH sides on the drive train
     *      - right if you the intake is facing forward and you are also facing forward
     *  - read the module roations from shuffle board or advantage scope
     *      - put these new module rotations here in the constants
     * 
     * - 
     * 
     */
    
    public static final double FL_OFFSET = 0.0583; // was -0.453711
    public static final double FR_OFFSET = 0.2138; // was 0.716553
    public static final double BL_OFFSET = -0.268; // was 0.230787455240885
    public static final double BR_OFFSET = 0.1169; // was -0.380286458333333

}

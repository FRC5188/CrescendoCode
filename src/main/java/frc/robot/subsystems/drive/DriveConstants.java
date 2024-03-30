package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

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
     *      - right is if you the intake is facing forward and you are also facing forward
     *  - read the module roations from shuffle board or advantage scope
     *      - put these new module rotations here in the constants
     * 
     * - redeploy code
     * 
     */
    
    public static final double FL_OFFSET = 0.0583; // was -0.453711
    public static final double FR_OFFSET = 0.2138; // was 0.716553
    public static final double BL_OFFSET = -0.268; // was 0.230787455240885
    public static final double BR_OFFSET = 0.1169; // was -0.380286458333333

    /*****************************
     * 
     * SYSID TEST TUNING
     * 
     *****************************/
    public static final Measure<Velocity<Voltage>> SYSID_RAMP_RRATE =
                        Volts.of(1.0).per(Seconds.of(1));
    public static final Measure<Voltage> SYSID_STEP_VOLTAGE = Volts.of(7);
    public static final Measure<Time> SYSID_TIMEOUT = Seconds.of(10);

    /*********************
     * 
     * MODULE FEEDFORWARD AND PID CONSTANTS
     * 
     **********************/
    // ka was 0.02617
    public static final double MODULE_FEEDFORWARD_KS = 0.074229; //0.1
    public static final double MODULE_FEEDFORWARD_KV = 0.10251; //0.13
    public static final double MODULE_FEEDFORWARD_KA = 0.02738; //
    public static final double MODULE_DRIVEPID_KP = 0.05; // was 0.05 // 0.14436 from sysid
    public static final double MODULE_DRIVEPID_KI = 0.0;
    public static final double MODULE_DRIVEPID_KD = 0.0;
    public static final double MODULE_TURNPID_KP = 7.0;
    public static final double MODULE_TURNPID_KI = 0.0;
    public static final double MODULE_TURNPID_KD = 0.0;

    // PID constants for auto aim command
    public static final double AUTO_ROTATE_P = 0.007;
    public static final double AUTO_ROTATE_I = 0.0001;
    public static final double AUTO_ROTATE_D = 0.0006;
    public static final double AUTO_ROTATE_TOLERANCE = 3.0;
    // PID constants for CmdShootOnTheMove
    public static final double SHOOT_ON_THE_MOVE_P = 0.13;
    public static final double SHOOT_ON_THE_MOVE_I = 0.003;
    public static final double SHOOT_ON_THE_MOVE_D = 0.00075;
    public static final double SHOOT_ON_THE_MOVE_TOLERANCE = 3.0;

    public static final double JOYSTICK_DEADBAND = 0.1;

    // Logs all of the IntakeConstants into Advantage Kit.
    static {
        Logger.recordOutput("Constants/Drive/MAX_LINEAR_SPEED", MAX_LINEAR_SPEED);
        Logger.recordOutput("Constants/Drive/TRACK_WIDTH_X", TRACK_WIDTH_X);
        Logger.recordOutput("Constants/Drive/TRACK_WIDTH_Y", TRACK_WIDTH_Y);
        Logger.recordOutput("Constants/Drive/DRIVE_BASE_RADIUS", DRIVE_BASE_RADIUS);
        Logger.recordOutput("Constants/Drive/MAX_ANGULAR_SPEED", MAX_ANGULAR_SPEED);
        
        Logger.recordOutput("Constants/Drive/RED_SPEAKER", RED_SPEAKER);
        Logger.recordOutput("Constants/Drive/BLUE_SPEAKER", BLUE_SPEAKER);
        
        Logger.recordOutput("Constants/Drive/DISCRETE_TIMESTEP", DISCRETE_TIMESTEP);
        
        Logger.recordOutput("Constants/Drive/FL_OFFSET", FL_OFFSET);
        Logger.recordOutput("Constants/Drive/FR_OFFSET", FR_OFFSET);
        Logger.recordOutput("Constants/Drive/BL_OFFSET", BL_OFFSET);
        Logger.recordOutput("Constants/Drive/BR_OFFSET", BR_OFFSET);
        
        Logger.recordOutput("Constants/Drive/SYSID_RAMP_RRATE", SYSID_RAMP_RRATE);
        Logger.recordOutput("Constants/Drive/SYSID_STEP_VOLTAGE", SYSID_STEP_VOLTAGE);
        Logger.recordOutput("Constants/Drive/SYSID_TIMEOUT", SYSID_TIMEOUT);

        Logger.recordOutput("Constants/Drive/MODULE_FEEDFORWARD_KS", MODULE_FEEDFORWARD_KS);
        Logger.recordOutput("Constants/Drive/MODULE_FEEDFORWARD_KV", MODULE_FEEDFORWARD_KV);
        Logger.recordOutput("Constants/Drive/MODULE_FEEDFORWARD_KA", MODULE_FEEDFORWARD_KA);
        Logger.recordOutput("Constants/Drive/MODULE_DRIVEPID_KP", MODULE_DRIVEPID_KP);
        Logger.recordOutput("Constants/Drive/MODULE_DRIVEPID_KI", MODULE_DRIVEPID_KI);
        Logger.recordOutput("Constants/Drive/MODULE_DRIVEPID_KD", MODULE_DRIVEPID_KD);
        Logger.recordOutput("Constants/Drive/MODULE_TURNPID_KP", MODULE_TURNPID_KP);
        Logger.recordOutput("Constants/Drive/MODULE_TURNPID_KI", MODULE_TURNPID_KI);
        Logger.recordOutput("Constants/Drive/MODULE_TURNPID_KD", MODULE_TURNPID_KD);

        Logger.recordOutput("Constants/Drive/AUTO_ROTATE_P", AUTO_ROTATE_P);
        Logger.recordOutput("Constants/Drive/AUTO_ROTATE_I", AUTO_ROTATE_I);
        Logger.recordOutput("Constants/Drive/AUTO_ROTATE_D", AUTO_ROTATE_D);
        Logger.recordOutput("Constants/Drive/AUTO_ROTATE_TOLERANCE", AUTO_ROTATE_TOLERANCE);

        Logger.recordOutput("Constants/Drive/SHOOT_ON_THE_MOVE_P", SHOOT_ON_THE_MOVE_P);
        Logger.recordOutput("Constants/Drive/SHOOT_ON_THE_MOVE_I", SHOOT_ON_THE_MOVE_I);
        Logger.recordOutput("Constants/Drive/SHOOT_ON_THE_MOVE_D", SHOOT_ON_THE_MOVE_D);
        Logger.recordOutput("Constants/Drive/SHOOT_ON_THE_MOVE_TOLERANCE", SHOOT_ON_THE_MOVE_TOLERANCE);

        Logger.recordOutput("Constants/Drive/JOYSTICK_DEADBAND", JOYSTICK_DEADBAND);
    }
}

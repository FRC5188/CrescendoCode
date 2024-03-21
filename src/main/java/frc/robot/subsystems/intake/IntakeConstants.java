package frc.robot.subsystems.intake;
import org.littletonrobotics.junction.Logger;

import frc.robot.util.LoggedTunableNumber;

public class IntakeConstants {

    /************************
     * 
     * INTAKE PID CONSTANTS
     * 
     *************************/
    // intake pivot
    // RETUNE THESE PID NUMBERS
    public static final LoggedTunableNumber PIVOT_PID_KP = new LoggedTunableNumber("Intake/Pivot/PID/kP", 0.0038);
    public static final LoggedTunableNumber PIVOT_PID_KI = new LoggedTunableNumber("Intake/Pivot/PID/kI", 0.000000);
    public static final LoggedTunableNumber PIVOT_PID_KD = new LoggedTunableNumber("Intake/Pivot/PID/kD", 0.000085);

    // 2/26/24-- P:0.005, I:0.00001, D:0.000045; MAX_VEL:30, MAX_ACCEL: 60, MAX_ISUM:0.1

    /***********************
     * 
     * INTAKE POSITION SETPOINTS
     * 
     * units: Degrees
     * larger angle is towards the ground. smaller angle is towards the robot
     * 
     **********************/
    public static final LoggedTunableNumber POSITION_SOURCE_PICKUP = new LoggedTunableNumber("Intake/Position/Source Pickup", 50);
    public static final LoggedTunableNumber POSITION_GROUND_PICKUP = new LoggedTunableNumber("Intake/Position/Ground Pickup", 188);
    public static final LoggedTunableNumber POSITION_STOWED = new LoggedTunableNumber("Intake/Position/Stowed", 12);
    public static final LoggedTunableNumber POSITION_AMP_SCORE = new LoggedTunableNumber("Intake/Position/Amp Score", 60);
    public static final LoggedTunableNumber POSITION_SPEAKER_SCORE = new LoggedTunableNumber("Intake/Position/Speaker Score", 115);

    /*************************
     * 
     * INTAKE MECHANICAL CONSTANTS
     * 
    ************************/
    public static final double INTAKE_WIDTH = 4; //units?
    public static final double INTAKE_HEIGHT = 3;
    public static final double INTAKE_LENGTH = 0.3;
    public static final double INTAKE_OFFSET_DEGREES = 136;// 57.2 degrees
    public static final double MIN_INTAKE_ANGLE = -2.0;
    public static final double MAX_INTAKE_ANGLE = 189.0;

    /***************************
     * 
     * INTAKE ROLLER SETTINGS
     * 
     **************************/
    public static final LoggedTunableNumber INTAKE_ACQUIRE_SPEED = new LoggedTunableNumber("Intake/Roller/Aquire Speed", 0.8);
    public static final LoggedTunableNumber INTAKE_SPIT_SPEED = new LoggedTunableNumber("Intake/Roller/Spit Speed", -0.7);
    public static final LoggedTunableNumber INTAKE_SPIT_TIME = new LoggedTunableNumber("Intake/Roller/Spit Time", 1.0);
    public static final LoggedTunableNumber INTAKE_CURRENT_CUTOFF = new LoggedTunableNumber("Intake/Roller/Current Cutoff", 40);

    // this constant allows us to make the rollers slowly spin to
    // keep hold of a game piece if we want to. Right now we dont want to
    public static final double INTAKE_STOP_SPEED = 0.00; 

    /*********************
     * 
     * INTAKE SOFTWARE CONSTANTS
     * 
     **********************/
    public static final double INTAKE_PIVOT_DEADBAND = 10;
    public static final int INTAKE_PIVOT_SMART_CURRENT_LIMIT = 40;
    public static final double INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT = 55;
    public static final int INTAKE_ROLLER_SMART_CURRENT_LIMIT = 50;
    public static final double INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT = 60;

    // Logs all of the IntakeConstants into Advantage Kit.
    
    static {

        Logger.recordOutput("Constants/Intake/INTAKE_WIDTH", INTAKE_WIDTH);
        Logger.recordOutput("Constants/Intake/INTAKE_HEIGHT", INTAKE_HEIGHT);
        Logger.recordOutput("Constants/Intake/INTAKE_LENGTH", INTAKE_LENGTH);
        Logger.recordOutput("Constants/Intake/INTAKE_OFFSET_DEGREES", INTAKE_OFFSET_DEGREES);
        Logger.recordOutput("Constants/Intake/MIN_INTAKE_ANGLE", MIN_INTAKE_ANGLE);
        Logger.recordOutput("Constants/Intake/MAX_INTAKE_ANGLE", MAX_INTAKE_ANGLE);

        Logger.recordOutput("Constants/Intake/INTAKE_STOP_SPEED", INTAKE_STOP_SPEED);
     
        Logger.recordOutput("Constants/Intake/INTAKE_PIVOT_DEADBAND", INTAKE_PIVOT_DEADBAND);
        Logger.recordOutput("Constants/Intake/INTAKE_PIVOT_SMART_CURRENT_LIMIT", INTAKE_PIVOT_SMART_CURRENT_LIMIT);
        Logger.recordOutput("Constants/Intake/INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT", INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT);
        Logger.recordOutput("Constants/Intake/INTAKE_ROLLER_SMART_CURRENT_LIMIT", INTAKE_ROLLER_SMART_CURRENT_LIMIT);
        Logger.recordOutput("Constants/Intake/INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT", INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT);
    }
}

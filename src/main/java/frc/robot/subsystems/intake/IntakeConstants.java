package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

public class IntakeConstants {

    /************************
     * 
     * INTAKE PID CONSTANTS
     * 
     *************************/
    // intake pivot
    // RETUNE THESE PID NUMBERS
    public static final double PIVOT_PID_KP = 0.005;
    public static final double PIVOT_PID_KI = 0.00001;
    public static final double PIVOT_PID_KD = 0.000045;
    public static final double PIVOT_PID_MAX_VEL = 30;
    public static final double PIVOT_PID_MAX_ACCEL = 60;
    public static final double PIVOT_PID_MAX_ISUM = 0.1;

    // 2/26/24-- P:0.005, I:0.00001, D:0.000045; MAX_VEL:30, MAX_ACCEL: 60, MAX_ISUM:0.1

    // intake rollers
    public static final double ROLLERS_PID_KP = 0.0;
    public static final double ROLLERS_PID_KI = 0.0;
    public static final double ROLLERS_PID_KD = 0.0;

    /***********************
     * 
     * INTAKE POSITION SETPOINTS
     * 
     * units: Degrees
     * larger angle is towards the ground. smaller angle is towards the robot
     * 
     **********************/
    public static final double POSITION_SOURCE_PICKUP = 50;
    public static final double POSITION_GROUND_PICKUP = 167;
    public static final double POSITION_STOWED = 12;
    public static final double POSITION_AMP_SCORE = 60;
    public static final double POSITION_SPEAKER_SCORE = 115;

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
    public static final double MAX_INTAKE_ANGLE = 180.0;

    /***************************
     * 
     * INTAKE ROLLER SETTINGS
     * 
     **************************/
    public static final double INTAKE_ACQUIRE_SPEED = 0.7;
    public static final double INTAKE_SPIT_SPEED = -0.7;
    public static final double INTAKE_SPIT_TIME = 1.0;
    public static final double INTAKE_CURRENT_CUTOFF = 40;
    // this constant allows us to make the rollers slowly spin to
    // keep hold of a game piece if we want to. Right now we dont want to
    public static final double INTAKE_STOP_SPEED = 0; 

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
}

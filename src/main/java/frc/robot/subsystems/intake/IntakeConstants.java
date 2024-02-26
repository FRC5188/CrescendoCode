package frc.robot.subsystems.intake;

public class IntakeConstants {
    /************************
     * 
     * INTAKE PID CONSTANTS
     * 
     *************************/
    // intake pivot
    public static final double PIVOT_PID_KP = 0.005;
    public static final double PIVOT_PID_KI = 0.000;
    public static final double PIVOT_PID_KD = 0.00;
    public static final double PIVOT_PID_MAX_VEL = 50;
    public static final double PIVOT_PID_MAX_ACCEL = 80;

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
    public static final double POSITION_GROUND_PICKUP = 175;
    public static final double POSITION_STOWED = 5;
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
    public static final double INTAKE_CURRENT_CUTOFF = 40;

    /*********************
     * 
     * INTAKE SOFTWARE CONSTANTS
     * 
     **********************/
    public static final double INTAKE_PIVOT_DEADBAND = 10;
    public static final int INTAKE_PIVOT_SMART_CURRENT_LIMIT = 40;
    public static final double INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT = 55;
    public static final int INTAKE_ROLLER_SMART_CURRENT_LIMIT = 40;
    public static final double INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT = 55;
}

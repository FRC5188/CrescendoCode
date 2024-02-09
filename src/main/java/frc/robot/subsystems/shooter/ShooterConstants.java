package frc.robot.subsystems.shooter;

public abstract class ShooterConstants {
    public static final double MAXIMUM_ANGLE_ENCODER_TURNS = 10;
    public static final double MINIMUM_ANGLE_ENCODER_TURNS = -10; // These might need changed later depending on how everything is implemented. 
    public static final double ANGLE_ENCODER_DEADBAND_DEGREES = 0.5; // This will be used when we've determining whether we can shoot or not. 
    public static final double ANGLE_ENCODER_OFFSET = 0;

    public static final double MIN_SHOOTER_ANGLE = 0;
    public static final double MAX_SHOOTER_ANGLE = 0;

    public static final double ANGLE_PID_P = 0;
    public static final double ANGLE_PID_I = 0;
    public static final double ANGLE_PID_D = 0;

    public static final double FLYWHEEL_GEAR_RATIO = 0;
    public static final double FLYWHEEL_SPEED_DEADBAND = 0;

    public static final double SHOOTER_WIDTH = 0;
    public static final double SHOOTER_HEIGHT = 0;
    public static final double SHOOTER_ROOT_WIDTH = 0;
    public static final double SHOOTER_ROOT_HEIGHT = 0;
    public static final double SHOOTER_LENGTH = 0;
    public static final double SHOOTER_ANGLE = 0;
}

package frc.robot.subsystems.shooter;

public abstract class ShooterConstants {
    public static final double MAXIMUM_ANGLE_ENCODER_ANGLE = 90;
    public static final double MINIMUM_ANGLE_ENCODER_ANGLE = 5; // These might need changed later depending on how everything is implemented. 
    public static final double ANGLE_ENCODER_DEADBAND_DEGREES = 1; // This will be used when we've determining whether we can shoot or not. 

    public static final double FLYWHEEL_GEAR_RATIO = 0;
    public static final double FLYWHEEL_SPEED_DEADBAND = 0;

    public static final double SHOOTER_WIDTH = 4;
    public static final double SHOOTER_HEIGHT = 3;
    public static final double SHOOTER_LENGTH = 0.3;
    public static final double SHOOTER_OFFSET_DEGREES = 0;
}

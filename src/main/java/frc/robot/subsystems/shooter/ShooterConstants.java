package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public abstract class ShooterConstants {

    /*********************
     * 
     * Shooter zone constants
     * 
     * flywheel speed: speed of both flywheels on the shooter in RPM
     * lower bound: distance to speaker in meters
     * upper bound: distance to speaker in meters
     * Shooter angle: Angle to set the shooter angle to in degrees.
     *                Higher angle moves the shooter towards vertical
     * 
     *********************/


    // podium
    public static final double ZONE_PODIUM_FLYWHEEL_SPEED = 2250;
    public static final double ZONE_PODIUM_LOW_BOUND = 4;
    public static final double ZONE_PODIUM_UPPER_BOUND = 2.5;
    public static final double ZONE_PODIUM_SHOOTER_ANGLE = 30;

    // subwoofer
    public static final double ZONE_SUBWOOFER_FLYWHEEL_SPEED = 1500;
    public static final double ZONE_SUBWOOFER_LOW_BOUND = 0.0;
    public static final double ZONE_SUBWOOFER_UPPER_BOUND = 2.5;
    public static final double ZONE_SUBWOOFER_SHOOTER_ANGLE = 41;

    // unkown
    public static final double ZONE_UNKOWN_FLYWHEEL_SPEED = 200;
    public static final double ZONE_UNKOWN_LOW_BOUND = -1;
    public static final double ZONE_UNKOWN_UPPER_BOUND = -1;
    public static final double ZONE_UNKOWN_SHOOTER_ANGLE = 35;


    /**************
     * 
     * SHOOTER ENCODER CONSTANTS
     *
     **************/

    public static final double MAXIMUM_ANGLE_ENCODER_ANGLE = 90;
    public static final double MINIMUM_ANGLE_ENCODER_ANGLE = 5; // These might need changed later depending on how everything is implemented. 
    public static final double ANGLE_ENCODER_DEADBAND_DEGREES = 1; // This will be used when we've determining whether we can shoot or not. 

    
    /***************
     * 
     * SHOOTER MECHANICAL CONSTANTS
     * 
     ***************/
    public static final double FLYWHEEL_GEAR_RATIO = 2;
    public static final double SHOOTER_CANVAS_WIDTH = 4; // units??
    public static final double SHOOTER_CANVAS_HEIGHT = 3;
    public static final double SHOOTER_HEIGHT_FROM_BASE = Units.inchesToMeters(20);
    public static final double SHOOTER_LENGTH = 0.3;
    public static final double SHOOTER_OFFSET_DEGREES = 0;

    /****************** 
     * SHOOTER SOFTWARE CONSTANTS
     * 
     *****************/
    public static final double FLYWHEEL_SPEED_DEADBAND = 250;

}

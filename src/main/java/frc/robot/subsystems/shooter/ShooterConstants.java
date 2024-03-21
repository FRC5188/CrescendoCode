package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;

public abstract class ShooterConstants {

    /*********************
     * 
     * Shooter zone constants
     * 
     * flywheel speed: speed of both flywheels on the shooter in RPM
     * lower bound: distance to speaker in meters
     * upper bound: distance to speaker in meters
     * Shooter angle: Angle to set the shooter angle to in degrees.
     * Higher angle moves the shooter towards vertical
     * 
     *********************/

    // PODIUM
    // public static final LoggedTunableNumber ZONE_PODIUM_FLYWHEEL_SPEED = new
    // LoggedTunableNumber("SHOOTER/PODIUM_FLYWHEEL_SPEED", 1750);
    public static final double ZONE_PODIUM_FLYWHEEL_SPEED = 1750;
    public static final double ZONE_PODIUM_LOW_BOUND = 4;
    public static final double ZONE_PODIUM_UPPER_BOUND = 2.5;
    public static final double ZONE_PODIUM_SHOOTER_ANGLE = 22;

    // SUBWOOFER
    // public static final LoggedTunableNumber ZONE_SUBWOOFER_FLYWHEEL_SPEED = new
    // LoggedTunableNumber("SHOOTER/SUBWOOFER_FLYWHEEL_SPEED", 1300);
    public static final double ZONE_SUBWOOFER_FLYWHEEL_SPEED = 1000;
    public static final double ZONE_SUBWOOFER_LOW_BOUND = 0.0;
    public static final double ZONE_SUBWOOFER_UPPER_BOUND = 2.5;
    public static final double ZONE_SUBWOOFER_SHOOTER_ANGLE = 42.0;

    // unknown
    public static final double ZONE_UNKNOWN_FLYWHEEL_SPEED = 200;
    public static final double ZONE_UNKNOWN_LOW_BOUND = -1;
    public static final double ZONE_UNKNOWN_UPPER_BOUND = -1;
    public static final double ZONE_UNKNOWN_SHOOTER_ANGLE = 35;

    /**************
     * 
     * SHOOTER FEEDER CONSTANTS
     *
     **************/
    public static final double FEEDER_SHOOT_SPEED = 0.7;
    public static final double FEEDER_PICKUP_SPEED = 0.00;

    /**************
     * 
     * SHOOTER ENCODER CONSTANTS
     *
     **************/
    public static final double MAXIMUM_ANGLE_ENCODER_ANGLE = 90; // These might need changed later depending on how everything is implemented. 
    public static final double MINIMUM_ANGLE_ENCODER_ANGLE = 10; // These might need changed later depending on how everything is implemented. 
    public static final double ANGLE_ENCODER_DEADBAND_DEGREES = 0.8; // This will be used when we've determining whether we can shoot or not. 
    
    /***************
     * 
     * SHOOTER VISUALIZATION CONSTANTS
     * 
     ***************/
    public static final double SHOOTER_CANVAS_WIDTH = 4; // units??
    public static final double SHOOTER_CANVAS_HEIGHT = 3;
    public static final double SHOOTER_HEIGHT_FROM_BASE = Units.inchesToMeters(20);
    public static final double SHOOTER_LENGTH = 0.3;
    public static final double SHOOTER_OFFSET_DEGREES = 0;

    /***************
     * 
     * SHOOTER MECHANICAL CONSTANTS
     * 
     ***************/
    public static final double FLYWHEEL_GEAR_RATIO = 2;
    public static final double ANGLE_GEAR_RATIO = 60;
    public static final double ANGLE_FROM_ROBOT_ZERO_TO_GROUND_DEGREES = 14.52;
    public static final double SHOOTER_MOMENT_ARM_LENGTH_METERS = 0.1219;
    public static final double SHOOTER_WEIGHT_KG = 5.222;
    public static final double MAX_VOLTAGE = 12.0;
    public static final double NEO_VORTEX_STALL_CURRENT = 211; // in amps
    public static final double NEO_VORTEX_STALL_TORQUE = 0.3671; // in kg*m
    public static final double NEO_VORTEX_INTERNAL_RESISTANCE = MAX_VOLTAGE / NEO_VORTEX_STALL_CURRENT;
    public static final double NEO_VORTEX_TORQUE_CONSTANT = NEO_VORTEX_STALL_TORQUE / NEO_VORTEX_STALL_CURRENT;
    public static final double SHOOTER_FEEDFORWARD_CONSTANT = 
        (SHOOTER_WEIGHT_KG * SHOOTER_MOMENT_ARM_LENGTH_METERS * NEO_VORTEX_INTERNAL_RESISTANCE) 
        / (NEO_VORTEX_TORQUE_CONSTANT * ANGLE_GEAR_RATIO * MAX_VOLTAGE);
    

    /******************
     * SHOOTER SOFTWARE CONSTANTS
     * 
     *****************/
    public static final double FLYWHEEL_SPEED_DEADBAND = 50;
    public static final double TIME_TO_SHOOT = 0.2; // In seconds 

    // Logs all of the ShooterConstants into Advantage Kit.
    static {
        Logger.recordOutput("Constants/Shooter/ZONE_PODIUM_FLYWHEEL_SPEED", ZONE_PODIUM_FLYWHEEL_SPEED);
        Logger.recordOutput("Constants/Shooter/ZONE_PODIUM_LOW_BOUND", ZONE_PODIUM_LOW_BOUND);
        Logger.recordOutput("Constants/Shooter/ZONE_PODIUM_UPPER_BOUND", ZONE_PODIUM_UPPER_BOUND);
        Logger.recordOutput("Constants/Shooter/ZONE_PODIUM_SHOOTER_ANGLE", ZONE_PODIUM_SHOOTER_ANGLE);

        Logger.recordOutput("Constants/Shooter/ZONE_SUBWOOFER_FLYWHEEL_SPEED", ZONE_SUBWOOFER_FLYWHEEL_SPEED);
        Logger.recordOutput("Constants/Shooter/ZONE_SUBWOOFER_LOW_BOUND", ZONE_SUBWOOFER_LOW_BOUND);
        Logger.recordOutput("Constants/Shooter/ZONE_SUBWOOFER_UPPER_BOUND", ZONE_SUBWOOFER_UPPER_BOUND);
        Logger.recordOutput("Constants/Shooter/ZONE_SUBWOOFER_SHOOTER_ANGLE", ZONE_SUBWOOFER_SHOOTER_ANGLE);

        Logger.recordOutput("Constants/Shooter/ZONE_UNKOWN_FLYWHEEL_SPEED", ZONE_UNKNOWN_FLYWHEEL_SPEED);
        Logger.recordOutput("Constants/Shooter/ZONE_UNKOWN_LOW_BOUND", ZONE_UNKNOWN_LOW_BOUND);
        Logger.recordOutput("Constants/Shooter/ZONE_UNKOWN_UPPER_BOUND", ZONE_UNKNOWN_UPPER_BOUND);
        Logger.recordOutput("Constants/Shooter/ZONE_UNKOWN_SHOOTER_ANGLE", ZONE_UNKNOWN_SHOOTER_ANGLE);

        Logger.recordOutput("Constants/Shooter/MAXIMUM_ANGLE_ENCODER_ANGLE", MAXIMUM_ANGLE_ENCODER_ANGLE);
        Logger.recordOutput("Constants/Shooter/MINIMUM_ANGLE_ENCODER_ANGLE", MINIMUM_ANGLE_ENCODER_ANGLE);
        Logger.recordOutput("Constants/Shooter/ANGLE_ENCODER_ANGLE_DEADBAND_DEGREES", ANGLE_ENCODER_DEADBAND_DEGREES);

        Logger.recordOutput("Constants/Shooter/FLYWHEEL_GEAR_RATIO", FLYWHEEL_GEAR_RATIO);
        Logger.recordOutput("Constants/Shooter/SHOOTER_CANVAS_WIDTH", SHOOTER_CANVAS_WIDTH);
        Logger.recordOutput("Constants/Shooter/SHOOTER_CANVAS_HEIGHT", SHOOTER_CANVAS_HEIGHT);
        Logger.recordOutput("Constants/Shooter/SHOOTER_HEIGHT_FROM_BASE", SHOOTER_HEIGHT_FROM_BASE);
        Logger.recordOutput("Constants/Shooter/SHOOTER_LENGTH", SHOOTER_LENGTH);
        Logger.recordOutput("Constants/Shooter/SHOOTER_OFFSET_DEGREES", SHOOTER_OFFSET_DEGREES);

        Logger.recordOutput("Constants/Shooter/FLYWHEEL_SPEED_DEADBAND", FLYWHEEL_SPEED_DEADBAND);
    }

}

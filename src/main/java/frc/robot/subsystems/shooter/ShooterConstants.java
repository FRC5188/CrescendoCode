package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

abstract class ShooterConstants {
    // ============= ZONE CONSTANTS =============
    // FLYWHEEL SPEED: Speed of both flywheels in RPM. (Rotations per Minute)
    // LOWER BOUND: Distance to speaker in meters.
    // UPPER BOUND: Distance to speaker in meters.
    // SHOOTER ANGLE: Angle to set the shooter angle to in degrees. Higher angle moves the shooter towards vertical.
    public abstract class ZONE {
        // PODIUM
        public static final LoggedTunableNumber ZONE_PODIUM_FLYWHEEL_SPEED = new LoggedTunableNumber("SHOOTER/Podium Flywheel Speed", 1750);
        public static final LoggedTunableNumber ZONE_PODIUM_LOW_BOUND = new LoggedTunableNumber("SHOOTER/Podium Low Bound", 4);
        public static final LoggedTunableNumber ZONE_PODIUM_UPPER_BOUND = new LoggedTunableNumber("SHOOTER/Podium Upper Bound", 2.5);
        public static final LoggedTunableNumber ZONE_PODIUM_SHOOTER_ANGLE = new LoggedTunableNumber("SHOOTER/Podium Shooter Angle", 22);

        // SUBWOOFER
        public static final LoggedTunableNumber ZONE_SUBWOOFER_FLYWHEEL_SPEED = new LoggedTunableNumber("SHOOTER/Subwoofer Flywheel Speed", 1300);
        public static final LoggedTunableNumber ZONE_SUBWOOFER_LOW_BOUND = new LoggedTunableNumber("SHOOTER/Subwoofer Low Bound", 0.0);
        public static final LoggedTunableNumber ZONE_SUBWOOFER_UPPER_BOUND = new LoggedTunableNumber("SHOOTER/Subwoofer Upper Bound", 2.5);
        public static final LoggedTunableNumber ZONE_SUBWOOFER_SHOOTER_ANGLE = new LoggedTunableNumber("SHOOTER/Subwoofer Shooter Angle", 40);

        // UNKNOWN
        public static final LoggedTunableNumber ZONE_UNKNOWN_FLYWHEEL_SPEED = new LoggedTunableNumber("SHOOTER/Unknown Flywheel Speed", 200);
        public static final LoggedTunableNumber ZONE_UNKNOWN_LOW_BOUND = new LoggedTunableNumber("SHOOTER/Unknown Low Bound", -1);
        public static final LoggedTunableNumber ZONE_UNKNOWN_UPPER_BOUND = new LoggedTunableNumber("SHOOTER/Unknown Upper Bound", -1);
        public static final LoggedTunableNumber ZONE_UNKNOWN_SHOOTER_ANGLE = new LoggedTunableNumber("SHOOTER/Unknown Shooter Angle", 35);
    }

    public abstract class ENCODER {
        public static final double MAXIMUM_ANGLE_ENCODER_ANGLE = 90;
        public static final double MINIMUM_ANGLE_ENCODER_ANGLE = 5; // These might need changed later depending on how everything is implemented. 
        public static final double ANGLE_ENCODER_DEADBAND_DEGREES = 1; // This will be used when we've determining whether we can shoot or not. 
    }

    public abstract class MECHANICAL {
        public static final double FLYWHEEL_GEAR_RATIO = 2;
        public static final double SHOOTER_CANVAS_WIDTH_METERS = 4;
        public static final double SHOOTER_CANVAS_HEIGHT_METERS = 3;
        public static final double SHOOTER_HEIGHT_FROM_BASE_METERS = Units.inchesToMeters(20);
        public static final double SHOOTER_LENGTH_METERS = 0.3;
        public static final double SHOOTER_OFFSET_DEGREES = 0;
    }
    
    public abstract class PID {
        public abstract class FLYHWEELS {
            public static final LoggedTunableNumber KP = new LoggedTunableNumber("SHOOTER/PID/FLYWHEELS/KP", 0.005);
            public static final LoggedTunableNumber KI = new LoggedTunableNumber("SHOOTER/PID/FLYWHEELS/KI", 0.00001);
            public static final LoggedTunableNumber KD = new LoggedTunableNumber("SHOOTER/PID/FLYWHEELS/KD", 0.000045);
            public static final LoggedTunableNumber KF = new LoggedTunableNumber("SHOOTER/PID/FLYWHEELS/KF", 0.00022);
            public static final double TOLERANCE = 50;
        }
        public abstract class PIVOT {
            public static final LoggedTunableNumber KP = new LoggedTunableNumber("SHOOTER/PID/PIVOT/KP", 0.025);
            public static final LoggedTunableNumber KI = new LoggedTunableNumber("SHOOTER/PID/PIVOT/KI", 0.0);
            public static final LoggedTunableNumber KD = new LoggedTunableNumber("SHOOTER/PID/PIVOT/KD", 0.0);
        }
    }
}

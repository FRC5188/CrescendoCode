package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public abstract class ShooterConstants {

    /**
     * <STRONG>Flywheel Speed: </STRONG> Speed of both flywheels measured in RPM (Rotations per Minute) </p>
     * <STRONG>Lower Bound: </STRONG> Closest distance to the Speaker where you're still in that zone in meters. </p>
     * <STRONG>Upper Bound: </STRONG> Farthest distance to the Speaker where you're still in that zone in meters. </p>
     * <STRONG>Shooter Angle: </STRONG> Angle to set the shooter angle to in degrees. Higher angle moves the shooter towards vertical.
     */
    public abstract class ZONE_CONSTANTS {
        public abstract class PODIUM {
            public static double FLYWHEEL_SPEED = 2250;
            public static double LOW_BOUND = 4;
            public static double UPPER_BOUND = 2.5;
            public static double SHOOTER_ANGLE = 30;

            public static LoggedTunableNumber FLYWHEEL_SPEED_TUNABLE = new LoggedTunableNumber("Shooter/Podium/Flywheel Speed", FLYWHEEL_SPEED);
            public static LoggedTunableNumber LOW_BOUND_TUNABLE = new LoggedTunableNumber("Shooter/Podium/Low Bound", LOW_BOUND);
            public static LoggedTunableNumber UPPER_BOUND_TUNABLE = new LoggedTunableNumber("Shooter/Podium/Upper Bound", UPPER_BOUND);
            public static LoggedTunableNumber SHOOTER_ANGLE_TUNABLE = new LoggedTunableNumber("Shooter/Podium/Shooter Angle", SHOOTER_ANGLE);
        }

        public abstract class SUBWOOFER {
            public static double FLYWHEEL_SPEED = 1500;
            public static double LOW_BOUND = 0.0;
            public static double UPPER_BOUND = 2.5;
            public static double SHOOTER_ANGLE = 41;

            public static LoggedTunableNumber FLYWHEEL_SPEED_TUNABLE = new LoggedTunableNumber("Shooter/Subwoofer/Flywheel Speed", FLYWHEEL_SPEED);
            public static LoggedTunableNumber LOW_BOUND_TUNABLE = new LoggedTunableNumber("Shooter/Subwoofer/Low Bound", LOW_BOUND);
            public static LoggedTunableNumber UPPER_BOUND_TUNABLE = new LoggedTunableNumber("Shooter/Subwoofer/Upper Bound", UPPER_BOUND);
            public static LoggedTunableNumber SHOOTER_ANGLE_TUNABLE = new LoggedTunableNumber("Shooter/Subwoofer/Shooter Angle", SHOOTER_ANGLE);
        }

        public abstract class UNKNOWN {
            public static double FLYWHEEL_SPEED = 200;
            public static double LOW_BOUND = -1;
            public static double UPPER_BOUND = -1;
            public static double SHOOTER_ANGLE = 35;

            public static LoggedTunableNumber FLYWHEEL_SPEED_TUNABLE = new LoggedTunableNumber("Shooter/Unknown/Flywheel Speed", FLYWHEEL_SPEED);
            public static LoggedTunableNumber LOW_BOUND_TUNABLE = new LoggedTunableNumber("Shooter/Unknown/Low Bound", LOW_BOUND);
            public static LoggedTunableNumber UPPER_BOUND_TUNABLE = new LoggedTunableNumber("Shooter/Unknown/Upper Bound", UPPER_BOUND);
            public static LoggedTunableNumber SHOOTER_ANGLE_TUNABLE = new LoggedTunableNumber("Shooter/Unknown/Shooter Angle", SHOOTER_ANGLE);
        }
    }

    /** PID Constants (kP, kI, kD) for the flywheels and pivot. */
    public abstract class PID {
        public abstract class FLYHWEEL {
            public static double KP = 0.0001;
            public static double KI = 0.0000;
            public static double KD = 0.0000;
            public static double KF = 0.00022;

            public static LoggedTunableNumber KP_TUNABLE = new LoggedTunableNumber("Shooter/Flywheel/kP", KP);
            public static LoggedTunableNumber KI_TUNABLE = new LoggedTunableNumber("Shooter/Flywheel/kI", KI);
            public static LoggedTunableNumber KD_TUNABLE = new LoggedTunableNumber("Shooter/Flywheel/kD", KD);
            public static LoggedTunableNumber KF_TUNABLE = new LoggedTunableNumber("Shooter/Flywheel/kF", KF);
        }

        public abstract class ANGLE {
            public static double KP = 0.0001;
            public static double KI = 0.0000;
            public static double KD = 0.0000;
            public static double KF = 0.00022;

            public static LoggedTunableNumber KP_TUNABLE = new LoggedTunableNumber("Shooter/Angle/kP", KP);
            public static LoggedTunableNumber KI_TUNABLE = new LoggedTunableNumber("Shooter/Angle/kI", KI);
            public static LoggedTunableNumber KD_TUNABLE = new LoggedTunableNumber("Shooter/Angle/kD", KD);
            public static LoggedTunableNumber KF_TUNABLE = new LoggedTunableNumber("Shooter/Angle/kF", KF);
        }
    }

    public abstract class MECHANICAL {
        public static double FLYWHEEL_GEAR_RATIO = 2;
        public static double SHOOTER_CANVAS_WIDTH = 4;
        public static double SHOOTER_CANVAS_HEIGHT = 3;
        public static double SHOOTER_HEIGHT_FROM_BASE = Units.inchesToMeters(20);
        public static double SHOOTER_LENGTH = 0.3;
        public static double SHOOTER_OFFSET_DEGREES = 0;
    }

    public abstract class SOFTWARE {
        public static double FLYWHEEL_SPEED_DEADBAND = 250;

        public static LoggedTunableNumber FLYWHEEL_SPEED_DEADBAND_TUNABLE = new LoggedTunableNumber("Shooter/Software/Flywheel Speed Deadband", FLYWHEEL_SPEED_DEADBAND);
    }

    public static double MAXIMUM_ANGLE_ENCODER_ANGLE = 90;
    public static double MINIMUM_ANGLE_ENCODER_ANGLE = 5;
    public static double ANGLE_ENCODER_DEADBAND_DEGREES = 1;
}

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.util.tunable.LoggedTunableNumber;

public abstract class ShooterConstants {

    /**
     * <STRONG>Flywheel Speed: </STRONG> Speed of both flywheels measured in RPM (Rotations per Minute) </p>
     * <STRONG>Lower Bound: </STRONG> Closest distance to the Speaker where you're still in that zone in meters. </p>
     * <STRONG>Upper Bound: </STRONG> Farthest distance to the Speaker where you're still in that zone in meters. </p>
     * <STRONG>Shooter Angle: </STRONG> Angle to set the shooter angle to in degrees. Higher angle moves the shooter towards vertical.
     */
    public abstract class ZONE_CONSTANTS {
        public abstract class PODIUM {
            public static LoggedTunableNumber FLYWHEEL_SPEED = new LoggedTunableNumber("Shooter/Podium/Flywheel-Speed");
            public static LoggedTunableNumber LOW_BOUND = new LoggedTunableNumber("Shooter/Podium/Low-Bound");
            public static LoggedTunableNumber UPPER_BOUND = new LoggedTunableNumber("Shooter/Podium/Upper-Bound");
            public static LoggedTunableNumber SHOOTER_ANGLE = new LoggedTunableNumber("Shooter/Podium/Shooter-Angle");

            static {
                if (Robot.isReal()) {
                    FLYWHEEL_SPEED.initDefault(2250); // FLYWHEEL SPEED
                    LOW_BOUND.initDefault(4); // LOWER BOUND
                    UPPER_BOUND.initDefault(2.5); // UPPER BOUND
                    SHOOTER_ANGLE.initDefault(30); // SHOOTER ANGLE
                }
                else if (Robot.isSimulation()) {
                    throw new UnsupportedOperationException("Robot Simulation isn't supported yet for Shooter");
                }
            }
        }

        public abstract class SUBWOOFER {
            public static LoggedTunableNumber FLYWHEEL_SPEED = new LoggedTunableNumber("Shooter/Subwoofer/Flywheel-Speed");
            public static LoggedTunableNumber LOW_BOUND = new LoggedTunableNumber("Shooter/Subwoofer/Low-Bound");
            public static LoggedTunableNumber UPPER_BOUND = new LoggedTunableNumber("Shooter/Subwoofer/Upper-Bound");
            public static LoggedTunableNumber SHOOTER_ANGLE = new LoggedTunableNumber("Shooter/Subwoofer/Shooter-Angle");

            static {
                if (Robot.isReal()) {
                    FLYWHEEL_SPEED.initDefault(1500); // FLYWHEEL SPEED
                    LOW_BOUND.initDefault(0.0); // LOWER BOUND
                    UPPER_BOUND.initDefault(2.5); // UPPER BOUND
                    SHOOTER_ANGLE.initDefault(41); // SHOOTER ANGLE
                }
                else if (Robot.isSimulation()) {
                    throw new UnsupportedOperationException("Robot Simulation isn't supported yet for Shooter");
                }
            }
        }

        public abstract class UNKNOWN {
            public static LoggedTunableNumber FLYWHEEL_SPEED = new LoggedTunableNumber("Shooter/Unknown/Flywheel-Speed");
            public static LoggedTunableNumber LOW_BOUND = new LoggedTunableNumber("Shooter/Unknown/Low-Bound");
            public static LoggedTunableNumber UPPER_BOUND = new LoggedTunableNumber("Shooter/Unknown/Upper-Bound");
            public static LoggedTunableNumber SHOOTER_ANGLE = new LoggedTunableNumber("Shooter/Unknown/Shooter-Angle");

            static {
                if (Robot.isReal()) {
                    FLYWHEEL_SPEED.initDefault(200); // FLYWHEEL SPEED
                    LOW_BOUND.initDefault(-1.0); // LOWER BOUND
                    UPPER_BOUND.initDefault(-1.0); // UPPER BOUND
                    SHOOTER_ANGLE.initDefault(35.0); // SHOOTER ANGLE
                }
                else if (Robot.isSimulation()) {
                    throw new UnsupportedOperationException("Robot Simulation isn't supported yet for Shooter");
                }
            }
        }
    }

    /** PID Constants (kP, kI, kD) for the Flywheels and Pivot. */
    public abstract class PID {
        public abstract class FLYHWEEL {
            public static LoggedTunableNumber KP = new LoggedTunableNumber("Shooter/Flywheel/kP");
            public static LoggedTunableNumber KI = new LoggedTunableNumber("Shooter/Flywheel/kI");
            public static LoggedTunableNumber KD = new LoggedTunableNumber("Shooter/Flywheel/kD");
            public static LoggedTunableNumber KF= new LoggedTunableNumber("Shooter/Flywheel/kF");

            static {
                if (Robot.isReal()) {
                    KP.initDefault(0.0001); // KP
                    KI.initDefault(0.0000); // KI
                    KD.initDefault(0.0000); // KD
                    KF.initDefault(0.00022); // KF
                }
                else if (Robot.isSimulation()) {
                    throw new UnsupportedOperationException("Robot Simulation isn't supported yet for Shooter");
                }
            }
        }

        public abstract class ANGLE {
            public static LoggedTunableNumber KP = new LoggedTunableNumber("Shooter/Angle/kP");
            public static LoggedTunableNumber KI = new LoggedTunableNumber("Shooter/Angle/kI");
            public static LoggedTunableNumber KD = new LoggedTunableNumber("Shooter/Angle/kD");
            public static LoggedTunableNumber KF = new LoggedTunableNumber("Shooter/Angle/kF");

            static {
                if (Robot.isReal()) {
                    KP.initDefault(0.0001); // KP
                    KI.initDefault(0.0); // KI
                    KD.initDefault(0.0); // KD
                    KF.initDefault(0.00022); // KF
                }
                else if (Robot.isSimulation()) {
                    throw new UnsupportedOperationException("Robot Simulation isn't supported yet for Shooter");
                }
            }
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
        public static LoggedTunableNumber FLYWHEEL_SPEED_DEADBAND= new LoggedTunableNumber("Shooter/Software/Flywheel Speed Deadband");

        static {
            if (Robot.isReal()) {
                FLYWHEEL_SPEED_DEADBAND.initDefault(250);
            }
            else if (Robot.isSimulation()) {
                throw new UnsupportedOperationException("Robot Simulation isn't supported yet for Shooter");
            }
        }
    }

    public static double MAXIMUM_ANGLE_ENCODER_ANGLE = 90;
    public static double MINIMUM_ANGLE_ENCODER_ANGLE = 5;
    public static double ANGLE_ENCODER_DEADBAND_DEGREES = 1;
}

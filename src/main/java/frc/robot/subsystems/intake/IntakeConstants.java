package frc.robot.subsystems.intake;

import frc.robot.util.LoggedTunableNumber;

public abstract class IntakeConstants {

    /************************
     * 
     * INTAKE PID CONSTANTS
     * 
     *************************/
    public abstract class PID {
        public abstract class PIVOT {
            // 2.26.24 -- P:0.005, I:0.00001, D:0.000045; MAX_VEL:30, MAX_ACCEL: 60, MAX_ISUM:0.1
            public static final LoggedTunableNumber KP = new LoggedTunableNumber("INTAKE/PID/PIVOT/KP", 0.0);
            public static final LoggedTunableNumber KI = new LoggedTunableNumber("INTAKE/PID/PIVOT/KI", 0.00001);
            public static final LoggedTunableNumber KD = new LoggedTunableNumber("INTAKE/PID/PIVOT/KD", 0.000045);
            public static final LoggedTunableNumber MAXIMUM_VELOCITY = new LoggedTunableNumber("INTAKE/PID/PIVOT/MAXIMUM_VELOCITY", 30);
            public static final LoggedTunableNumber MAXIMUM_ACCELERATION = new LoggedTunableNumber("INTAKE/PID/PIVOT/MAXIMUM_ACCELERATION", 60);
            public static final LoggedTunableNumber MAXIMUM_INTEGRAL_SUM = new LoggedTunableNumber("INTAKE/PID/PIVOT/MAXIMUM_INTEGRAL_SUM", 0.1);
        }

        public abstract class ROLLERS {
            public static final LoggedTunableNumber KP = new LoggedTunableNumber("INTAKE/PID/ROLLERS/KP", 0.0);
            public static final LoggedTunableNumber KI = new LoggedTunableNumber("INTAKE/PID/ROLLERS/KI", 0.0);
            public static final LoggedTunableNumber KD = new LoggedTunableNumber("INTAKE/PID/ROLLERS/KD", 0.0);
        }
    }

    public abstract class POSITIONS {
        public static final LoggedTunableNumber SOURCE_PICKUP = new LoggedTunableNumber("INTAKE/POSITIONS/SOURCE_PICKUP", 50);
        public static final LoggedTunableNumber GROUND_PICKUP = new LoggedTunableNumber("INTAKE/POSITIONS/GROUND_PICKUP", 167);
        public static final LoggedTunableNumber STOWED = new LoggedTunableNumber("INTAKE/POSITIONS/STOWED", 12);
        public static final LoggedTunableNumber AMP_SCORE = new LoggedTunableNumber("INTAKE/POSITIONS/AMP_SCORE", 60);
        public static final LoggedTunableNumber SPEAKER_SCORE = new LoggedTunableNumber("INTAKE/POSITIONS/SPEAKER_SCORE", 115);
    }

    public abstract class MECHANICAL {
        public static final double WIDTH = 4;
        public static final double HEIGHT = 3;
        public static final double LENGTH = 0.3;
        public static final double OFFSET_DEGREES = 136;
        public static final double MIN_ANGLE = -2.0;
        public static final double MAX_ANGLE = 180.0;
    }

    public abstract class ROLLERS {
        public static final LoggedTunableNumber AQUIRE_SPEED = new LoggedTunableNumber("INTAKE/ROLLERS/ACQUIRE_SPEED", 0.7);
        public static final LoggedTunableNumber SPIT_SPEED = new LoggedTunableNumber("INTAKE/ROLLERS/SPIT_SPEED", -0.7);
        public static final LoggedTunableNumber SPIT_TIME = new LoggedTunableNumber("INTAKE/ROLLERS/SPIT_TIME", 1.0);
        public static final LoggedTunableNumber CURRENT_CUTOFF = new LoggedTunableNumber("INTAKE/ROLLERS/CURRENT_CUTOFF", 40);
        public static final LoggedTunableNumber STOP_SPEED = new LoggedTunableNumber("INTAKE/ROLLERS/STOP_SPEED", 0);
    }

    public abstract class SOFTWARE {
        public static final double PIVOT_DEADBAND = 10;
        public static final int PIVOT_CURRENT_LIMIT = 40;
        public static final double PIVOT_SECONDARY_CURRENT_LIMIT = 55;
        public static final int ROLLER_CURRENT_LIMIT = 50;
        public static final double ROLLER_SECONDARY_CURRENT_LIMIT = 60;
    }
}

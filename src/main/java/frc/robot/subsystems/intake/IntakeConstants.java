package frc.robot.subsystems.intake;

import frc.robot.util.LoggedTunableNumber;

public abstract class IntakeConstants {
    public abstract class PID {
        public abstract class PIVOT {
            public static double KP = 0.005;
            public static double KI = 0.000;
            public static double KD = 0.00;
            public static double KF = 0.00;

            public static final double PIVOT_PID_MAX_VEL = 50;
            public static final double PIVOT_PID_MAX_ACCEL = 80;

            public static LoggedTunableNumber KP_TUNABLE = new LoggedTunableNumber("Intake/Pivot/kP", KP);
            public static LoggedTunableNumber KI_TUNABLE = new LoggedTunableNumber("Intake/Pivot/kI", KI);
            public static LoggedTunableNumber KD_TUNABLE = new LoggedTunableNumber("Intake/Pivot/kD", KD);
            public static LoggedTunableNumber KF_TUNABLE = new LoggedTunableNumber("Intake/Pivot/kF", KF);
        }
        public abstract class ROLLERS {
            public static double KP = 0.0;
            public static double KI = 0.0;
            public static double KD = 0.0;
            public static double KF = 0.0;

            public static LoggedTunableNumber KP_TUNABLE = new LoggedTunableNumber("Intake/Rollers/kP", KP);
            public static LoggedTunableNumber KI_TUNABLE = new LoggedTunableNumber("Intake/Rollers/kI", KI);
            public static LoggedTunableNumber KD_TUNABLE = new LoggedTunableNumber("Intake/Rollers/kD", KD);
            public static LoggedTunableNumber KF_TUNABLE = new LoggedTunableNumber("Intake/Rollers/kF", KF);
        }
    }

    public abstract class POSITIONS {
        public static final double POSITION_SOURCE_PICKUP_DEFAULT = 50;
        public static final double POSITION_GROUND_PICKUP_DEFAULT = 175;
        public static final double POSITION_STOWED_DEFAULT = 5;
        public static final double POSITION_AMP_SCORE_DEFAULT = 60;
        public static final double POSITION_SPEAKER_SCORE_DEFAULT = 115;

        public static final LoggedTunableNumber POSITION_SOURCE_PICKUP = new LoggedTunableNumber("Intake/Position_Source_Pickup", POSITION_SOURCE_PICKUP_DEFAULT);
        public static final LoggedTunableNumber POSITION_GROUND_PICKUP = new LoggedTunableNumber("Intake/Position_Ground_Pickup", POSITION_GROUND_PICKUP_DEFAULT);
        public static final LoggedTunableNumber POSITION_STOWED = new LoggedTunableNumber("Intake/Position_Stowed", POSITION_STOWED_DEFAULT);
        public static final LoggedTunableNumber POSITION_AMP_SCORE = new LoggedTunableNumber("Intake/Position_Amp_Score", POSITION_AMP_SCORE_DEFAULT);
        public static final LoggedTunableNumber POSITION_SPEAKER_SCORE = new LoggedTunableNumber("Intake/Position_Speaker_Score", POSITION_SPEAKER_SCORE_DEFAULT);
    }

    public abstract class MECHANICAL {
        public static final double PIVOT_LENGTH = 0.3;
        public static final double PIVOT_OFFSET_DEGREES = 136;
        public static final double PIVOT_WIDTH = 4;
        public static final double PIVOT_HEIGHT = 3;
        public static final double PIVOT_MIN_ANGLE = -2.0;
        public static final double PIVOT_MAX_ANGLE = 180.0;

        public static final double INTAKE_WIDTH = 4; //units?
        public static final double INTAKE_HEIGHT = 3;
        public static final double INTAKE_LENGTH = 0.3;
        public static final double INTAKE_OFFSET_DEGREES = 136;// 57.2 degrees
        public static final double MIN_INTAKE_ANGLE = -2.0;
        public static final double MAX_INTAKE_ANGLE = 180.0;
    }

    public abstract class INTAKE {
        public static final double INTAKE_ACQUIRE_SPEED_DEFAULT = 0.7;
        public static final double INTAKE_SPIT_SPEED_DEFAULT = -0.7;
        public static final double INTAKE_CURRENT_CUTOFF_DEFAULT = 40;

        public static final LoggedTunableNumber INTAKE_ACQUIRE_SPEED = new LoggedTunableNumber("Intake/Acquire_Speed", INTAKE_ACQUIRE_SPEED_DEFAULT);
        public static final LoggedTunableNumber INTAKE_SPIT_SPEED = new LoggedTunableNumber("Intake/Spit_Speed", INTAKE_SPIT_SPEED_DEFAULT);
        public static final LoggedTunableNumber INTAKE_CURRENT_CUTOFF = new LoggedTunableNumber("Intake/Current_Cutoff", INTAKE_CURRENT_CUTOFF_DEFAULT);
    }

    public abstract class SOFTWARE {
        public static final double INTAKE_PIVOT_DEADBAND = 10;
        public static final int INTAKE_PIVOT_SMART_CURRENT_LIMIT_DEFAULT = 40;
        public static final double INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT_DEFAULT = 55;
        public static final int INTAKE_ROLLER_SMART_CURRENT_LIMIT_DEFAULT = 40;
        public static final double INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT_DEFAULT = 55;

        public static final LoggedTunableNumber INTAKE_PIVOT_SMART_CURRENT_LIMIT = new LoggedTunableNumber("Intake/Pivot_Smart_Current_Limit", INTAKE_PIVOT_SMART_CURRENT_LIMIT_DEFAULT);
        public static final LoggedTunableNumber INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT = new LoggedTunableNumber("Intake/Pivot_Secondary_Current_Limit", INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT_DEFAULT);
        public static final LoggedTunableNumber INTAKE_ROLLER_SMART_CURRENT_LIMIT = new LoggedTunableNumber("Intake/Roller_Smart_Current_Limit", INTAKE_ROLLER_SMART_CURRENT_LIMIT_DEFAULT);
        public static final LoggedTunableNumber INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT = new LoggedTunableNumber("Intake/Roller_Secondary_Current_Limit", INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT_DEFAULT);
    }


    /***********************
     * 
     * INTAKE POSITION SETPOINTS
     * 
     * units: Degrees
     * larger angle is towards the ground. smaller angle is towards the robot
     * 
     **********************/

    /*************************
     * 
     * INTAKE MECHANICAL CONSTANTS
     * 
    ************************/


    /***************************
     * 
     * INTAKE ROLLER SETTINGS
     * 
     **************************/

    /*********************
     * 
     * INTAKE SOFTWARE CONSTANTS
     * 
     **********************/

}

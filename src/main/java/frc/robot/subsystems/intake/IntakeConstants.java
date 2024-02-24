package frc.robot.subsystems.intake;

import frc.robot.Robot;
import frc.robot.util.tunable.LoggedTunableNumber;

/** These are the constants that are used in the Intake.java implementation. The guide toward using this file is outline below which sets rules for adding and changing values inside.*/

// RULES & GUIDELINES FOR ADDING CONSTANTS
// 1.) Please assure that whenever you add something it's properly categorized. For instance if something doesn't have to do with the PID don't put it in it's subclass.
// 2.) Please assure that units are added to your constant. If unitless this isn't a problem. 
// 3.) If you're adding a constant that doesn't have an implementation then use -1.0 as the default value. Our code should throw an expcetion if it gets used anywhere.

// RULES & GUIDELINES FOR CHANGING CONSTANTS
// 1.) For changing the default value of a constant such as PID then look in the .initDefault() method, this is where these are stored.
public abstract class IntakeConstants {
    /**<STRONG>Purpose:</STRONG> Tuning PIDs on Intake. (Pivot & Rollers) */
    public abstract class PID {
        public abstract class PIVOT {
            public static final LoggedTunableNumber KP = new LoggedTunableNumber("Intake/Pivot/kP");
            public static final LoggedTunableNumber KI = new LoggedTunableNumber("Intake/Pivot/kI");
            public static final LoggedTunableNumber KD = new LoggedTunableNumber("Intake/Pivot/kD");
            public static final LoggedTunableNumber KF = new LoggedTunableNumber("Intake/Pivot/kF");
            public static final LoggedTunableNumber PIVOT_PID_MAX_VELOCITY_METERS_SECOND = new LoggedTunableNumber("Intake/Pivot/Pivot_PID_Max_Velocity");
            public static final LoggedTunableNumber PIVOT_PID_MAX_ACCCELERATION_METERS_SECOND_SQUARE = new LoggedTunableNumber("Intake/Pivot/Pivot_PID_Max_Acceleration");

            static {
                // Since we might want to use simulations at some point this will let us dynamically change the values depending on whether we're real or in simulation.
                // This will be used for all of the constants.
                if (Robot.isReal()) {
                    KP.initDefault(0.005); // KP
                    KI.initDefault(0.000); // KI
                    KD.initDefault(0.00); // KD
                    KF.initDefault(0.00); // KF
                    PIVOT_PID_MAX_VELOCITY_METERS_SECOND.initDefault(50); // Maximum Velocity (m/s)
                    PIVOT_PID_MAX_ACCCELERATION_METERS_SECOND_SQUARE.initDefault(80); // Maximum Acceleration (m/s^2)
                }
                else if (Robot.isSimulation()){
                    throw new UnsupportedOperationException("Robot Simulation isn't supported yet for Intake");
                }
            }
        }

        public abstract class ROLLERS {
            public static LoggedTunableNumber KP = new LoggedTunableNumber("Intake/Rollers/kP");
            public static LoggedTunableNumber KI = new LoggedTunableNumber("Intake/Rollers/kI");
            public static LoggedTunableNumber KD = new LoggedTunableNumber("Intake/Rollers/kD");
            public static LoggedTunableNumber KF = new LoggedTunableNumber("Intake/Rollers/kF");

            static {
                if (Robot.isReal()) {
                    KP.initDefault(0.005); // KP
                    KI.initDefault(0.000); // KI
                    KD.initDefault(0.00); // KD
                    KF.initDefault(0.00); // KF
                }
                else if (Robot.isSimulation()){
                    throw new UnsupportedOperationException("Robot Simulation isn't supported yet for Intake");
                }
            }
        }
    }

    /**<STRONG>Purpose:</STRONG> Angles of Intake in degrees as part of our cycle.*/
    public abstract class POSITIONS {
        public static final LoggedTunableNumber POSITION_SOURCE_PICKUP_DEGREES = new LoggedTunableNumber("Intake/Position_Source_Pickup");
        public static final LoggedTunableNumber POSITION_GROUND_PICKUP_DEGREES = new LoggedTunableNumber("Intake/Position_Ground_Pickup");
        public static final LoggedTunableNumber POSITION_STOWED_DEGREES = new LoggedTunableNumber("Intake/Position_Stowed");
        public static final LoggedTunableNumber POSITION_AMP_SCORE_DEGREES = new LoggedTunableNumber("Intake/Position_Amp_Score");
        public static final LoggedTunableNumber POSITION_SPEAKER_SCORE_DEGREES = new LoggedTunableNumber("Intake/Position_Speaker_Score");

        static {
            if (Robot.isReal()) {
                POSITION_SOURCE_PICKUP_DEGREES.initDefault(50); // SOURCE PICKUP
                POSITION_GROUND_PICKUP_DEGREES.initDefault(175); // GROUND PICKUP
                POSITION_STOWED_DEGREES.initDefault(5); // STOWED
                POSITION_AMP_SCORE_DEGREES.initDefault(60); // AMP SCORE
                POSITION_SPEAKER_SCORE_DEGREES.initDefault(115); // SPEAKER SCORE
            }
            else if (Robot.isSimulation()){
                throw new UnsupportedOperationException("Robot Simulation isn't supported yet for Intake");
            }
        }
    }

    public abstract class MECHANICAL {
        // Since these are mechanical constants and therefore won't be tuned they will remain unchanged.
        public static final double PIVOT_LENGTH_METERS = 0.3;
        public static final double PIVOT_OFFSET_DEGREES = 136;
        public static final double PIVOT_WIDTH_METERS = 4;
        public static final double PIVOT_HEIGHT_METERS = 3;
        public static final double PIVOT_MIN_ANGLE_DEGREES = -2.0;
        public static final double PIVOT_MAX_ANGLE_DEGREES = 180.0;

        public static final double INTAKE_WIDTH_METERS = 4;
        public static final double INTAKE_HEIGHT_METERS = 3;
        public static final double INTAKE_LENGTH_METERS = 0.3;
        public static final double INTAKE_OFFSET_DEGREES = 136;
        public static final double MIN_INTAKE_ANGLE_DEGREES = -2.0;
        public static final double MAX_INTAKE_ANGLE_DEGREES = 180.0;
    }

    public abstract class INTAKE {
        public static final LoggedTunableNumber INTAKE_ACQUIRE_SPEED = new LoggedTunableNumber("Intake/Acquire_Speed");
        public static final LoggedTunableNumber INTAKE_SPIT_SPEED = new LoggedTunableNumber("Intake/Spit_Speed");
        public static final LoggedTunableNumber INTAKE_CURRENT_CUTOFF = new LoggedTunableNumber("Intake/Current_Cutoff");
        public static final LoggedTunableNumber INTAKE_IDLE_SPEED = new LoggedTunableNumber("Intake/Idle_Speed");

        static {
            if (Robot.isReal()) {
                INTAKE_ACQUIRE_SPEED.initDefault(0.7); // INTAKE AQCUIRE SPEED
                INTAKE_SPIT_SPEED.initDefault(-0.7); // INTAKE SPIT SPEED
                INTAKE_CURRENT_CUTOFF.initDefault(40); // INTAKE CURRENT CUTOFF
                INTAKE_IDLE_SPEED.initDefault(0.02); // INTAKE IDLE SPEED
            }
            else if (Robot.isSimulation()){
                throw new UnsupportedOperationException("Robot Simulation isn't supported yet for Intake");
            }
        }
    }

    public abstract class SOFTWARE {
        public static final LoggedTunableNumber INTAKE_PIVOT_DEADBAND_DEGREES = new LoggedTunableNumber("Intake/Pivot_Deadband_Degrees");
        public static final LoggedTunableNumber INTAKE_PIVOT_SMART_CURRENT_LIMIT = new LoggedTunableNumber("Intake/Pivot_Smart_Current_Limit");
        public static final LoggedTunableNumber INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT = new LoggedTunableNumber("Intake/Pivot_Secondary_Current_Limit");
        public static final LoggedTunableNumber INTAKE_ROLLER_SMART_CURRENT_LIMIT = new LoggedTunableNumber("Intake/Roller_Smart_Current_Limit");
        public static final LoggedTunableNumber INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT = new LoggedTunableNumber("Intake/Roller_Secondary_Current_Limit");

        static {
            if (Robot.isReal()) {
                INTAKE_PIVOT_DEADBAND_DEGREES.initDefault(10); // INTAKE PIVOT DEADBAND
                INTAKE_PIVOT_SMART_CURRENT_LIMIT.initDefault(40); // INTAKE PIVOT SMART CURRENT LIMIT
                INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT.initDefault(55); // INTAKE PIVOT SECONDARY CURRENT LIMIT
                INTAKE_ROLLER_SMART_CURRENT_LIMIT.initDefault(40); // INTAKE ROLLER SMART CURRENT LIMIT
                INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT.initDefault(55); // INTAKE ROLLER SECONDARY CURRENT LIMIT
            }
            else if (Robot.isSimulation()){
                throw new UnsupportedOperationException("Robot Simulation isn't supported yet for Intake");
            }
        }
    }
}

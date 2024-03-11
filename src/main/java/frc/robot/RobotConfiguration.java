package frc.robot;

import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.RealClimberIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.RealIntakeIO;
import frc.robot.subsystems.shooter.RealShooterIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.vision.RealVisionIO;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.visiondrive.RealVisionDriveIO;
import frc.robot.subsystems.visiondrive.VisionDriveIO;

/** Configurations used for turning ON/OFF various features that we might not want to use at a given time. */
public abstract class RobotConfiguration {

    // ROBOT MODE
    public static final boolean IS_REAL = true;
    public static final boolean IS_SIMULATED = !IS_REAL;

    // TUNING MODE FOR LOGGED TUNABLED INPUTS
    public static final boolean IS_IN_TUNING_MODE = false;

    // SUBSYSTEM-SPECIFIC CONFIGURATIONS
    public static final boolean IS_USING_SHOOTER = true; // Should be noted that we're assuming that the motor is still connected. We just don't run them.
    public static final boolean IS_USING_INTAKE = true;
    public static final boolean IS_USING_CLIMBER = false;
    public static final boolean IS_USING_DRIVE_TRAIN = true;
    public static final boolean IS_USING_LIMELIGHT = true;
    public static final boolean IS_USING_VISION_ODOMETRY = true;

    // USED FOR CONFIGURATION OF THE TYPE OF ROBOT THAT WE'RE USING.
    public static Drive configureDrive() {
        if (IS_REAL) {
            return new Drive(
                    new GyroIONavX2(), // WE'RE USING THE NAVX GYROSCOPE
                    new RealVisionIO(), 
                    new RealVisionDriveIO(), 
                    new ModuleIOSparkFlex(0),
                    new ModuleIOSparkFlex(1),
                    new ModuleIOSparkFlex(2),
                    new ModuleIOSparkFlex(3));
        }

        // TODO: Implement the simulated version of the Drive class.
        else if (IS_SIMULATED) {
            return new Drive(
                new GyroIO() {}, 
                new VisionIO() {}, 
                new VisionDriveIO() {}, 
                new ModuleIOSim(),
                new ModuleIOSim(), 
                new ModuleIOSim(), 
                new ModuleIOSim()
                );
        }

        else throw new IllegalStateException("RobotConfiguration [DRIVE]: Invalid Robot Mode.");
    }

    public static Shooter configureShooter() {
        if (IS_REAL) {
            return new Shooter(new RealShooterIO());
        }
        else if (IS_SIMULATED) {
            return new Shooter(new ShooterIO() {});
        }

        else throw new IllegalStateException("RobotConfiguration [SHOOTER]: Invalid Robot Mode.");
    }

    public static Intake configureIntake() {
        if (IS_REAL) {
            return new Intake(new RealIntakeIO());
        }
        else if (IS_SIMULATED) {
            return new Intake(new IntakeIO() {});
        }

        else throw new IllegalStateException("RobotConfiguration [INTAKE]: Invalid Robot Mode.");
    }

    public static Climber configureClimber() {
        if (IS_REAL) {
            return new Climber(new RealClimberIO());
        }
        else if (IS_SIMULATED) {
            return new Climber(new ClimberIO() {});
        }

        else throw new IllegalStateException("RobotConfiguration [CLIMBER]: Invalid Robot Mode.");
    }
}

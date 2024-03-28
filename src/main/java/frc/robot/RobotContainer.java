// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.RealIntakeIO;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;
import frc.robot.subsystems.drive.commands.CmdDriveAutoAim;
import frc.robot.subsystems.drive.commands.DriveCommands;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.multisubsystemcommands.CmdAdjustShooterAutomatically;
import frc.robot.subsystems.multisubsystemcommands.CmdGoToNote;
import frc.robot.subsystems.multisubsystemcommands.GrpShootNoteInZone;
import frc.robot.subsystems.shooter.RealShooterIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;
import frc.robot.subsystems.shooter.commands.CmdShooterWaitUntilReady;
import frc.robot.subsystems.vision.RealVisionIO;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.visiondrive.RealVisionDriveIO;
import frc.robot.subsystems.visiondrive.VisionDrive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems
        private final Drive _drive;
        private final Intake _intake;
        private final Shooter _shooter;
        // private final Climber _climber;
        private final VisionDrive _visionDrive;

        private double _driveMultiplier = 1.0;

        private Command _adjustShooterAutomaticallyCommand;

        // logged dashboard inputs
        private final LoggedDashboardChooser<Command> _autoChooser;

        // Controller
        private final CommandXboxController _driveController = new CommandXboxController(0);

        // Button box
        // Top half of buttons
        private final GenericHID _operatorController1 = new GenericHID(1);
        // Bottom half of buttons
        private final GenericHID _operatorController2 = new GenericHID(2);

        // Top row, left to right
        private JoystickButton _opButtonOne = new JoystickButton(_operatorController1, 1);
        private JoystickButton _opButtonTwo = new JoystickButton(_operatorController1, 2);
        private JoystickButton _opButtonThree = new JoystickButton(_operatorController1, 3);

        // second row from top. left to right
        private JoystickButton _opButtonFour = new JoystickButton(_operatorController1, 4);
        private JoystickButton _opButtonFive = new JoystickButton(_operatorController1, 5);
        private JoystickButton _opButtonSix = new JoystickButton(_operatorController1, 6);

        // Third row from top. left to right
        private JoystickButton _opButtonSeven = new JoystickButton(_operatorController1, 7);
        private JoystickButton _opButtonEight = new JoystickButton(_operatorController1, 8);
        private JoystickButton _opButtonNine = new JoystickButton(_operatorController1, 9);

        // Top left toggle switch
        private JoystickButton _autoShootToggle = new JoystickButton(_operatorController1, 10);

        // fourth from top row. left to right
        private JoystickButton _op2ButtonOne = new JoystickButton(_operatorController2, 1);
        private JoystickButton _op2ButtonTwo = new JoystickButton(_operatorController2, 2);
        private JoystickButton _op2ButtonThree = new JoystickButton(_operatorController2, 3);

        // second from the bottom row. left to right
        private JoystickButton _op2ButtonFour = new JoystickButton(_operatorController2, 4);
        private JoystickButton _op2ButtonFive = new JoystickButton(_operatorController2, 5);
        private JoystickButton _op2ButtonSix = new JoystickButton(_operatorController2, 6);

        // bottom row. left to right
        private JoystickButton _op2ButtonSeven = new JoystickButton(_operatorController2, 7);
        private JoystickButton _op2ButtonEight = new JoystickButton(_operatorController2, 8);
        private JoystickButton _op2ButtonNine = new JoystickButton(_operatorController2, 9);
        
        private JoystickButton _op2ButtonTen = new JoystickButton(_operatorController2, 10);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.CURRENT_MODE) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                _drive = new Drive(
                                                new GyroIONavX2(),
                                                new RealVisionIO(),
                                                new ModuleIOSparkFlex(0),
                                                new ModuleIOSparkFlex(1),
                                                new ModuleIOSparkFlex(2),
                                                new ModuleIOSparkFlex(3));
                                _intake = new Intake(new RealIntakeIO());
                                _shooter = new Shooter(new RealShooterIO());
                                // _climber = new Climber(new RealClimberIO());
                                _visionDrive = new VisionDrive(new RealVisionDriveIO());
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                _drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new VisionIO() {
                                                },
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim());
                                _intake = new Intake(new IntakeIO() {
                                });
                                _shooter = new Shooter(new ShooterIO() {
                                });
                                // _climber = new Climber(new RealClimberIO() {
                                // });
                                _visionDrive = new VisionDrive(new RealVisionDriveIO() {
                                });
                                break;
                        default:
                                // Replayed robot, disable IO implementations
                                _drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new VisionIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                });
                                _intake = new Intake(new IntakeIO() {
                                });
                                _shooter = new Shooter(new ShooterIO() {
                                });
                                // _climber = new Climber(new RealClimberIO() {
                                // });
                                _visionDrive = new VisionDrive(new RealVisionDriveIO() {
                                });
                                break;
                }

                // setup hand-scheduled commands
                _adjustShooterAutomaticallyCommand = new CmdAdjustShooterAutomatically(_drive, _shooter, _intake);

                NamedCommands.registerCommand("Pickup_Note_Without_Limelight", _intake.buildCommand().pickUpFromGround(4000));
                NamedCommands.registerCommand("Pickup_Note_With_Limelight", new PrintCommand("[ERROR] Not implemented"));
                NamedCommands.registerCommand("Shooting_From_Subwoofer", new GrpShootNoteInZone(_intake, _shooter, ShooterZone.Subwoofer));
                NamedCommands.registerCommand("Shooting_From_Podium", new GrpShootNoteInZone(_intake, _shooter, ShooterZone.Podium));

                DoubleSupplier zeroSupplier = new DoubleSupplier() {
                                        @Override
                                        public double getAsDouble() {
                                            return 0;
                                        }
                                };

                NamedCommands.registerCommand("Auto_Shoot_With_Auto_Align", 
                        new SequentialCommandGroup(
                                _shooter.buildCommand().setAutoShootEnabled(true),
                                new CmdDriveAutoAim(_drive, zeroSupplier, zeroSupplier),
                                new CmdShooterWaitUntilReady(_shooter).withTimeout(2),
                                _intake.buildCommand().spit(2)
                        ));
                _autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
                // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                /*
                 * ================================
                 * Drive Controller
                 * ================================
                 */
                _drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                _drive,
                                                () -> -_driveController.getLeftY()*_driveMultiplier,
                                                () -> -_driveController.getLeftX()*_driveMultiplier,
                                                () -> -_driveController.getRightX()*_driveMultiplier));
                // create an x shaped pattern with the wheels to make it harder to push us
                // _driveController.x().onTrue(Commands.runOnce(_drive::stopWithX, _drive));

                // face the speaker while we hold this button
                // _driveController.leftBumper().whileTrue(new CmdShootOnTheMove(
                // _drive,
                // _shooter,
                // _intake,
                // () -> _driveController.getRightTriggerAxis(),
                // () -> _driveController.getLeftX(),
                // () -> _driveController.getRightX()));
                _driveController.leftBumper().whileTrue(new CmdDriveAutoAim(_drive,
                        () -> -_driveController.getLeftY()*_driveMultiplier,
                        () -> -_driveController.getLeftX()*_driveMultiplier));      
                

                _driveController.a().whileTrue(
                        new InstantCommand(
                                () -> this._driveMultiplier = 0.6
                        )
                );

                _driveController.a().whileFalse(
                        new InstantCommand(
                                () -> this._driveMultiplier = 1.0
                        )
                );

                _driveController.rightBumper().whileTrue(new CmdGoToNote(_drive, _visionDrive));


                // reset the orientation of the robot. changes which way it thinks is forward
                _driveController.y().onTrue(
                                Commands.runOnce(
                                                () -> _drive.setPose(
                                                                new Pose2d(_drive.getPose().getTranslation(),
                                                                                new Rotation2d(Math.PI))),
                                                _drive).ignoringDisable(true));

                double inchesFromSubwoofer = 39.0;
                double robotWidth = 13.0 + 1.5;
                Pose2d robotOnSubwooferRed = new Pose2d(
                                DriveConstants.RED_SPEAKER.getX()
                                                - Units.inchesToMeters(inchesFromSubwoofer + robotWidth),
                                DriveConstants.RED_SPEAKER.getY(),
                                new Rotation2d(Math.PI));
                // Change the robot pose to think it is in front of the red speaker
                _driveController.b().onTrue(
                                Commands.runOnce(
                                () -> _drive.setPose(robotOnSubwooferRed), _drive).ignoringDisable(true));

                
                /*
                 * ================================
                 * Button Box
                 * ================================
                 */

                // The toggle sends out true when we want to disable autoshoot, so this looks a
                // little flipped around
                _autoShootToggle.onFalse(Commands.runOnce(() -> _shooter.setAutoShootEnabled(true)))
                                .onTrue(Commands.runOnce(() -> _shooter.setAutoShootEnabled(false)));

                // Adjust shooter angle from current position
                _opButtonOne.onTrue(this._shooter.buildCommand().adjustAngle(1));
                _opButtonFour.onTrue(this._shooter.buildCommand().adjustAngle(-1));

                // Move intake to different positions
                _opButtonNine.onTrue(_intake.buildCommand().spit(IntakeConstants.INTAKE_SPIT_TIME.get()));
                _op2ButtonNine.onTrue(Commands.runOnce(() -> _intake.resetHasNote()));
                _op2ButtonSix.onTrue(Commands.runOnce(() -> _intake.setHasNote()));
                _opButtonFive.onTrue(this._intake.buildCommand().setPosition(IntakePosition.Stowed));
                _opButtonEight.onTrue(this._intake.buildCommand().pickUpFromGround(0));
                
                // Run intake rollers, stop when we let go of button
                _opButtonSeven.onTrue(this._intake.buildCommand().acquire())
                        .onFalse(this._intake.buildCommand().stop());

                _opButtonTwo.onTrue(new GrpShootNoteInZone(_intake, _shooter, ShooterZone.Amp));

                // Move to shooter positions manually
                _op2ButtonFive.onTrue(new GrpShootNoteInZone(_intake, _shooter, ShooterZone.Subwoofer));
                _op2ButtonEight.onTrue(new GrpShootNoteInZone(_intake, _shooter, ShooterZone.Podium));

               
                // sad face button
                _op2ButtonThree.onTrue(new InstantCommand(
                                () -> Logger.recordOutput(":(", true)));

                _op2ButtonThree.onFalse(new InstantCommand(
                                () -> Logger.recordOutput(":(", false)));

                /***
                 * 
                 * SYSID BUTTON MAPPINGS
                 * These lines map the sysid routines to the d-pad on the driver's controller.
                 * These should NEVER
                 * be enabled for a match. Read the links below for more about how to use sysid
                 * 
                 * - how the code is setup (that's what these lines below do)
                 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html#creating-an-identification-routine
                 * 
                 * - running the routines
                 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/running-routine.html#running-the-identification-routine
                 * 
                 * - looking at the data
                 * https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html
                 * - note: use advantagescope to connect from the robot and download a log file
                 * with the data
                 * - you can use the position, velocity, and voltage of one module to represent
                 * the whole drivetrain
                 * 
                 * - take the feedforward and feedback numbers and put them in
                 * driveconstants.java
                 */
                // _controller.povRight().whileTrue(_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                // _controller.povLeft().whileTrue(_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                // _controller.povUp().whileTrue(_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                // _controller.povDown().whileTrue(_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return _autoChooser.get();
        }

        public Command getAdjustShooterAutomaticallyCommand() {
                return this._adjustShooterAutomaticallyCommand;
        }

        public Command getRunAnglePIDCommand() {
                return _shooter.buildCommand().runAnglePID();
        }

        public Command getSetInitalShooterPosition() {
                return _shooter.buildCommand().setPositionByZone(ShooterZone.Unknown);
        }

        public Command getFeederInitialStateCommand() {
                return Commands.runOnce(() -> _intake.setFeederMotorPickupSpeed());
        }
}

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

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;
import frc.robot.subsystems.drive.commands.CmdDriveRotateAboutSpeaker;
import frc.robot.subsystems.drive.commands.DriveCommands;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.commands.CmdIntakeRollersAcquire;
import frc.robot.subsystems.intake.commands.CmdIntakeRunPID;
import frc.robot.subsystems.intake.commands.CmdIntakeSetPosition;
import frc.robot.subsystems.intake.commands.CmdIntakeStopRollers;
import frc.robot.subsystems.intake.commands.GrpIntakeAcquireNoteFromGround;
import frc.robot.subsystems.intake.commands.GrpIntakeAcquireNoteFromSource;
import frc.robot.subsystems.intake.commands.GrpIntakeMoveToPosition;
import frc.robot.subsystems.multisubsystemcommands.CmdRunShooterAutomatically;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.CmdShooterRunPids;
import frc.robot.hardware.intake.IntakeIO;
import frc.robot.hardware.intake.RealIntakeIO;
import frc.robot.hardware.shooter.RealShooterIO;
import frc.robot.hardware.shooter.ShooterIO;
import frc.robot.hardware.vision.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

    // Controller
    private final CommandXboxController _controller = new CommandXboxController(0);

    // Button box
    // Top row of buttons
    private final Joystick _operatorController1 = new Joystick(1);

    // Bottom row of buttons
    // private final Joystick _operatorController2 = new Joystick(2);

    // Left column, top to bottom
    // private JoystickButton _opButtonOne = new
    // JoystickButton(_operatorController1, 1);
    // private JoystickButton _opButtonTwo = new
    // JoystickButton(_operatorController1, 2);
    // private JoystickButton _opButtonThree = new
    // JoystickButton(_operatorController1, 3);

    // Middle column, top to bottom
    private JoystickButton _opButtonFour = new JoystickButton(_operatorController1, 4);
    private JoystickButton _opButtonFive = new JoystickButton(_operatorController1, 5);
    private JoystickButton _opButtonSix = new JoystickButton(_operatorController1, 6);

    // Right column, top to bottom
    // private JoystickButton _opButtonSeven = new
    // JoystickButton(_operatorController1, 7);
    // private JoystickButton _opButtonEight = new
    // JoystickButton(_operatorController1, 8);
    private JoystickButton _opButtonNine = new JoystickButton(_operatorController1, 9);

    // Side Toggle Switch
    // private JoystickButton _opButtonTen = new
    // JoystickButton(_operatorController1, 10);

    // Bottom rows, left to right (not top then bottom!)
    // private JoystickButton _op2ButtonOne = new
    // JoystickButton(_operatorController2, 1);
    // private JoystickButton _op2ButtonTwo = new
    // JoystickButton(_operatorController2, 2);
    // private JoystickButton _op2ButtonThree = new
    // JoystickButton(_operatorController2, 3);
    // private JoystickButton _op2ButtonFour = new
    // JoystickButton(_operatorController2, 4);
    // private JoystickButton _op2ButtonFive = new
    // JoystickButton(_operatorController2, 5);
    // private JoystickButton _op2ButtonSix = new
    // JoystickButton(_operatorController2, 6);

    // Bottom right button (Frowny face)
    // private JoystickButton _op2ButtonNine = new
    // JoystickButton(_operatorController2, 9);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> _autoChooser;
    /*
     * private final LoggedDashboardNumber flywheelSpeedInput =
     * new LoggedDashboardNumber("Flywheel Speed", 1500.0);
     */

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
                _intake = new Intake(
                        new RealIntakeIO());
                _shooter = new Shooter(
                    new RealShooterIO());

                // flywheel = new Flywheel(new FlywheelIOSparkMax());
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
                _intake = new Intake(
                        new IntakeIO() {
                        });
                _shooter = new Shooter(
                        new ShooterIO() {
                        });
                // flywheel = new Flywheel(new FlywheelIOSim());
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
                _intake = new Intake(
                        new IntakeIO() {
                        });
                _shooter = new Shooter(
                        new ShooterIO() {
                        });

                // flywheel = new Flywheel(new FlywheelIO() {});
                break;

        }

        // Set up auto routines
        /*
         * NamedCommands.registerCommand(
         * "Run Flywheel",
         * Commands.startEnd(
         * () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop,
         * flywheel)
         * .withTimeout(5.0));
         */
        _autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        _autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                _drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        _autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                _drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        _autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", _drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        _autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", _drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        /*
         * autoChooser.addOption(
         * "Flywheel SysId (Quasistatic Forward)",
         * flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
         * autoChooser.addOption(
         * "Flywheel SysId (Quasistatic Reverse)",
         * flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
         * autoChooser.addOption(
         * "Flywheel SysId (Dynamic Forward)",
         * flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
         * autoChooser.addOption(
         * "Flywheel SysId (Dynamic Reverse)",
         * flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
         */

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
        _drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        _drive,
                        () -> -_controller.getLeftY() * .60,
                        () -> -_controller.getLeftX() * .60,
                        () -> _controller.getRightX() * .65));
        _controller
                .b()
                .whileTrue(new CmdDriveRotateAboutSpeaker(_drive,
                        () -> -_controller.getLeftY(),
                        () -> -_controller.getLeftX()));

        // -- Operator Controls --

        // _opButtonOne.onTrue();
        // _opButtonTwo.onTrue();
        // _opButtonThree.onTrue();

        // Source Position
        _opButtonFour.onTrue(new GrpIntakeAcquireNoteFromSource(_intake, 0));

        // Stow
        _opButtonFive.onTrue(new GrpIntakeMoveToPosition(_intake, IntakePosition.Stowed));

        // Ground Pickup Position
        _opButtonSix.onTrue(new GrpIntakeAcquireNoteFromGround(_intake, 0));

        // _opButtonSeven.onTrue();
        // _opButtonEight.onTrue();

        _opButtonNine.onTrue(new CmdIntakeRollersAcquire(_intake));
        _opButtonNine.onFalse(new CmdIntakeStopRollers(_intake));

        // Autoshoot toggle switch
        // _opButtonTen.onTrue();
        // _opButtonTen.onFalse();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return _autoChooser.get();
    }

    public Command getAutoShootCommand() {
        return new CmdRunShooterAutomatically(_drive, _shooter, _intake);
    }

    public Command getIntakeRunPIDCommand() {
        return new CmdIntakeRunPID(_intake);
    }

    public Command getShooterRunPIDCommand() {
        return new CmdShooterRunPids(_shooter);
    }

    public Command getIntakeSetStowed() {
        return new CmdIntakeSetPosition(_intake, IntakePosition.Stowed);
    }
}

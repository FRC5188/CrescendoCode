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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.intake.IntakeIO;
import frc.robot.hardware.intake.RealIntakeIO;
import frc.robot.hardware.shooter.RealShooterIO;
import frc.robot.hardware.shooter.ShooterIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;
import frc.robot.subsystems.drive.commands.CmdDriveRotateAboutSpeaker;
import frc.robot.subsystems.drive.commands.DriveCommands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.commands.CmdIntakeRollersSpit;
import frc.robot.subsystems.intake.commands.CmdIntakeRunPID;
import frc.robot.subsystems.intake.commands.CmdIntakeSetPosition;
import frc.robot.subsystems.intake.commands.GrpIntakeAcquireNoteFromGround;
import frc.robot.subsystems.multisubsystemcommands.CmdRunShooterAutomatically;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.CmdShooterRunPids;

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
                        new ModuleIOSparkFlex(0),
                        new ModuleIOSparkFlex(1),
                        new ModuleIOSparkFlex(2),
                        new ModuleIOSparkFlex(3));
                _intake = new Intake(new RealIntakeIO());
                _shooter = new Shooter(new RealShooterIO());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                _drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                _intake = new Intake(new IntakeIO(){});
                _shooter = new Shooter(new ShooterIO(){});
                break;

            default:
                // Replayed robot, disable IO implementations
                _drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                _intake = new Intake(new IntakeIO(){});
                _shooter = new Shooter(new ShooterIO(){});
                break;
        }

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
                        () -> -_controller.getLeftY(),
                        () -> -_controller.getLeftX(),
                        () -> _controller.getRightX()));
        //_controller.x().onTrue(Commands.runOnce(_drive::stopWithX, _drive));
        _controller
                .leftBumper()
                .whileTrue(new CmdDriveRotateAboutSpeaker(_drive,
                        () -> -_controller.getLeftY(),
                        () -> -_controller.getLeftX()));

        _controller.a().onTrue(new CmdIntakeSetPosition(_intake, IntakePosition.SourcePickup));
        _controller.b().onTrue(new GrpIntakeAcquireNoteFromGround(_intake, 0));
        
        // _controller.b().onTrue(new CmdIntakeRollersAcquire(_intake));

        _controller.x().onTrue(new CmdIntakeRollersSpit(_intake));
        _controller.y().onTrue(new CmdIntakeSetPosition(_intake, IntakePosition.Stowed));
        // _controller.b().whileTrue(new CmdShooterRunPids(_shooter));
        
        // _controller
        //         .a()
        //         .whileTrue(new CmdShooterSetAutoshootEnabled(_shooter, true))
        //         .whileFalse(new CmdShooterSetAutoshootEnabled(_shooter, false));
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

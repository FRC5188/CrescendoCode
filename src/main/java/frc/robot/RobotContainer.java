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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.vision.RealVisionIO;
import frc.robot.hardware.vision.VisionIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;
import frc.robot.subsystems.drive.commands.CmdDriveRotateAboutSpeaker;
import frc.robot.subsystems.drive.commands.DriveCommands;

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
    // private final Flywheel flywheel;

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
                        new RealVisionIO(),
                        new ModuleIOSparkFlex(0),
                        new ModuleIOSparkFlex(1),
                        new ModuleIOSparkFlex(2),
                        new ModuleIOSparkFlex(3));
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
                        () -> -_controller.getLeftY(),
                        () -> _controller.getLeftX(),
                        () -> -_controller.getRightX()));
        _controller.x().onTrue(Commands.runOnce(_drive::stopWithX, _drive));
        _controller
                .b()
                .whileTrue(new CmdDriveRotateAboutSpeaker(_drive,
                        () -> -_controller.getLeftY(),
                        () -> _controller.getLeftX()));
        /*
         * controller
         * .a()
         * .whileTrue(
         * Commands.startEnd(
         * () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop,
         * flywheel));
         */
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return _autoChooser.get();
    }
}

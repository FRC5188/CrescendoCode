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

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommandFactory;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.RealIntakeIO;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;
import frc.robot.subsystems.drive.commands.CmdDriveGoToNote;
import frc.robot.subsystems.drive.commands.CmdDriveRotateAboutSpeaker;
import frc.robot.subsystems.drive.commands.DriveCommands;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.multisubsystemcommands.GrpShootNoteInZone;
import frc.robot.subsystems.shooter.RealShooterIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;
import frc.robot.subsystems.shooter.commands.CmdShooterRunPids;
import frc.robot.subsystems.shooter.commands.CmdShooterSetPositionByZone;
import frc.robot.subsystems.vision.RealVisionIO;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.visiondrive.RealVisionDriveIO;
import frc.robot.subsystems.visiondrive.VisionDriveIO;

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
    private static final boolean USES_JOYSTICK = true;

    private static final short DRIVE_STATION_PORT = 0;

    // Subsystems
    private final Drive _drive;
    private final Intake _intake;
    private final Shooter _shooter;
    private ShooterZone _zone;
    private Command _runShooterPIDCommand;
    private Command _runIntakePIDCommand;

    // logged dashboard inputs
    private final LoggedDashboardChooser<Command> _autoChooser;

    // Controller
    private static Joystick _joystick = null;
    // If we use the flight joystick then we'll also need to make the buttons for it.
    private static JoystickButton _driveButtonOne = null;
    private static JoystickButton _driveButtonTwo = null;
    private static JoystickButton _driveButtonThree = null;
    private static JoystickButton _driveButtonFour = null;
    private static JoystickButton _driveButtonFive = null;
    private static JoystickButton _driveButtonSix = null;
    private static JoystickButton _driveButtonSeven = null;
    private static JoystickButton _driveButtonEight = null;
    private static JoystickButton _driveButtonNine = null;
    private static JoystickButton _driveButtonTen = null;
    private static JoystickButton _driveButtonEleven = null;
    private static JoystickButton _driveButtonTwelve = null;
    // We'll make then null for now so we don't waste space with random object and if the joystick is used then we'll give them them the value.

    private static CommandXboxController _controller = null;

    static {
        if (USES_JOYSTICK) {
            _joystick = new Joystick(DRIVE_STATION_PORT);

            // Now we'll make an object. 
            _driveButtonOne = new JoystickButton(_joystick, 1);
            _driveButtonTwo = new JoystickButton(_joystick, 2);
            _driveButtonThree = new JoystickButton(_joystick, 3);
            _driveButtonFour = new JoystickButton(_joystick, 4);
            _driveButtonFive = new JoystickButton(_joystick, 5);
            _driveButtonSix = new JoystickButton(_joystick, 6);
            _driveButtonSeven = new JoystickButton(_joystick, 7);
            _driveButtonEight = new JoystickButton(_joystick, 8);
            _driveButtonNine = new JoystickButton(_joystick, 9);
            _driveButtonTen = new JoystickButton(_joystick, 10);
            _driveButtonEleven = new JoystickButton(_joystick, 11);
            _driveButtonTwelve = new JoystickButton(_joystick, 12);
        }
        else {
            _controller = new CommandXboxController(DRIVE_STATION_PORT);
        }
    }

    // Button box
    // Top half of buttons
    private final Joystick _operatorController1 = new Joystick(1);

    // Bottom half of buttons
    private final Joystick _operatorController2 = new Joystick(2);

    // Left column, top to bottom
    // private JoystickButton _opButtonOne = new
    // JoystickButton(_operatorController1, 1);
    // private JoystickButton _opButtonTwo = new
    // JoystickButton(_operatorController1, 2);
    private JoystickButton _opButtonThree = new JoystickButton(_operatorController1, 3);

    // Middle column, top to bottom
    // private JoystickButton _opButtonFour = new
    // JoystickButton(_operatorController1, 4);
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
    private JoystickButton _op2ButtonOne = new JoystickButton(_operatorController2, 1);
    private JoystickButton _op2ButtonTwo = new JoystickButton(_operatorController2, 2);
    private JoystickButton _op2ButtonThree = new JoystickButton(_operatorController2, 3);
    private JoystickButton _op2ButtonFour = new JoystickButton(_operatorController2, 4);
    private JoystickButton _op2ButtonFive = new JoystickButton(_operatorController2, 5);
    private JoystickButton _op2ButtonSix = new JoystickButton(_operatorController2, 6);

    // this button is broken
    private JoystickButton _op2ButtonEight = new JoystickButton(_operatorController2, 8);

    // Bottom right button (Frowny face)
    private JoystickButton _op2ButtonNine = new JoystickButton(_operatorController2, 9);

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
                        new RealVisionDriveIO(),
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
                        new GyroIO() {},
                        new VisionIO() {},
                        new VisionDriveIO() {},
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                _intake = new Intake(new IntakeIO() {
                });
                _shooter = new Shooter(new ShooterIO() {
                });
                break;
            default:
                // Replayed robot, disable IO implementations
                _drive = new Drive(
                        new GyroIO() {},
                        new VisionIO() {},
                        new VisionDriveIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {
                        });
                _intake = new Intake(new IntakeIO() {
                });
                _shooter = new Shooter(new ShooterIO() {
                });
                break;
        }

        // setup commands for PID
        this._runShooterPIDCommand = new CmdShooterRunPids(_shooter);
        this._runIntakePIDCommand = _intake.buildCommand().runPID();

        NamedCommands.registerCommand("intake GroundPos", new IntakeCommandFactory(_intake).setPosition(IntakePosition.GroundPickup));
        NamedCommands.registerCommand("intake Stow", new IntakeCommandFactory(_intake).setPosition(IntakePosition.Stowed));
        NamedCommands.registerCommand("Subwoofer Shoot", new GrpShootNoteInZone(_intake, _shooter, ShooterZone.Subwoofer));
        NamedCommands.registerCommand("Podium Shoot", new GrpShootNoteInZone(_intake, _shooter, ShooterZone.Podium));
        NamedCommands.registerCommand("set has note", new Command() {
            @Override
            public void initialize() {
                _intake.setHasNote();
            }
            @Override
            public boolean isFinished() {
                return true;
            }
        });

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
        _drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        _drive,
                        () -> -_controller.getLeftY(),
                        () -> -_controller.getLeftX(),
                        () -> -_controller.getRightX()));
        // create an x shaped pattern with the wheels to make it harder to push us
        _controller.x().onTrue(Commands.runOnce(_drive::stopWithX, _drive));

        // face the speaker while we hold this button
        _controller.b().whileTrue(new CmdDriveRotateAboutSpeaker(_drive,
                () -> -_controller.getLeftY(),
                () -> -_controller.getLeftX()));

        _controller.a().whileTrue(new CmdDriveGoToNote(_drive));

        // reset the orientation of the robot. changes which way it thinks is forward
        _controller.y().onTrue(
                Commands.runOnce(
                        () -> _drive.setPose(
                                new Pose2d(_drive.getPose().getTranslation(), new Rotation2d(Math.PI))),
                        _drive).ignoringDisable(true));

        double inchesFromSubwoofer = 39.0;
        double robotWidth = 13.0 + 1.5;
        Pose2d robotOnSubwoofer = new Pose2d(
                DriveConstants.RED_SPEAKER.getX() - Units.inchesToMeters(inchesFromSubwoofer + robotWidth),
                DriveConstants.RED_SPEAKER.getY(),
                new Rotation2d(Math.PI));
        _controller.b().onTrue(
                Commands.runOnce(
                        () -> _drive.setPose(robotOnSubwoofer), _drive).ignoringDisable(true));

        // Move the shooter to the podium or subwoofer positions
        /*
         * ---------------- START MANUAL ROBOT CONTROL BUTTON
         * BINDINGS--------------------------
         */
        // consider adding a boolean to constants.java to put the robot into "pit" mode
        // or something to
        // switch the buttons to manual control for testing.

        // shooter position angle manual control
        _op2ButtonTwo.onTrue(new CmdShooterSetPositionByZone(_shooter, ShooterZone.Podium));
        _op2ButtonOne.onTrue(new CmdShooterSetPositionByZone(_shooter, ShooterZone.Subwoofer));
      
        // shooter fly wheel manual control. Only sets the flywheel speed while holding
        // the button
        //_op2ButtonFour.whileTrue(new CmdShooterRunFlywheelsForZone(_shooter, ShooterZone.Podium));
        //_op2ButtonThree.whileTrue(new CmdShooterRunFlywheelsForZone(_shooter, ShooterZone.Subwoofer));

        // FROM MAIN
        _opButtonFive.onTrue(this._intake.buildCommand().setPosition(IntakePosition.Stowed));
        _opButtonSix.onTrue(this._intake.buildCommand().pickUpFromGround());
        _opButtonThree.onTrue(this._intake.buildCommand().setPosition(IntakePosition.AmpScore));

        _opButtonNine.onTrue(this._intake.buildCommand().acquire());
        _opButtonNine.onFalse(this._intake.buildCommand().stop());

        _op2ButtonTwo.onTrue(new GrpShootNoteInZone(_intake, _shooter, ShooterZone.Podium));
        _op2ButtonOne.onTrue(new GrpShootNoteInZone(_intake, _shooter, ShooterZone.Subwoofer));

        _op2ButtonSix.onTrue(new Command() {

            @Override
            public void initialize(){
                _shooter.setFlywheelSpeed(0.0);
            }

            @Override
            public boolean isFinished(){
                return true;
            }
        }
        );

        _op2ButtonEight.onTrue(this._intake.buildCommand().spit(IntakeConstants.INTAKE_SPIT_TIME));

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

    public Command getRunShooterPIDCommand() {
        return this._runShooterPIDCommand;
    }

    public Command getRunIntakePIDCommand() {
        return this._runIntakePIDCommand;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// -----------------------------------------------------
// Thank you to Team 3467 - Windham Windup!
// Code Adapted from 3467's velocityOffset.java
// https://github.com/WHS-FRC-3467/Skip-5.14-Nocturne/blob/PostGSDCleanup/src/main/java/frc/robot/Commands/velocityOffset.java
// -----------------------------------------------------

package frc.robot.subsystems.multisubsystemcommands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class CmdShootOnTheMove extends Command {
  /** Creates a new CmdDriveShootOnTheMove. */
  private final Drive _drive;
  private final Shooter _shooter;
  private final Intake _intake;

  private DoubleSupplier _translationXSupplier;
  private DoubleSupplier _translationYSupplier;

  private Command _autoAdjustCommand;

  private boolean _isFinished;
  private boolean _cmdIsRunning;

  private Translation2d _currentRobotTranslation;
  private double _currentAngleRadians;

  private Translation2d _futureRobotTranslation;
  private Rotation2d _futureAngleToSpeaker;

  private ChassisSpeeds _speeds;
  private double _correctedRotation;
  private double _timeUntilShot;
  private Translation2d _moveDelta;
  private DoubleSupplier _trigger;
  private Timer _shotTimer;
  private Boolean _hasRunOnce;
  private double _correctedRadius;
  private Pose2d _futureRobotPose2d;
  private double _triggerThreshold;

  private PIDController _rotationPID;

  /**
   * CmdAdjustShooterAutomatically is the default command for the shooter. 
   * Disable it when we run this command.
   * 
   * @param drivetrainSubsystem  the drive subsystem
   * @param shooterSubsystem     the shooter subsystem
   * @param intakeSubsystem      the intake subsystem
   * @param triggerAxis          button binding used
   * @param translationXSupplier translation x supplied by driver translation
   *                             joystick
   * @param translationYSupplier translation y supplied by driver translation
   *                             joystick
[]\
   **/
  public CmdShootOnTheMove(Drive drivetrainSubsystem,
      Shooter shooterSubsystem,
      Intake intakeSubsystem,
      DoubleSupplier triggerAxis,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {

    _drive = drivetrainSubsystem;
    _shooter = shooterSubsystem;
    _intake = intakeSubsystem;
    _trigger = triggerAxis;
    _translationXSupplier = translationXSupplier;
    _translationYSupplier = translationYSupplier;
    //_autoAdjustCommand = autoAdjustCommand;
    _cmdIsRunning = true;

    _rotationPID = new PIDController(
        DriveConstants.SHOOT_ON_THE_MOVE_P,
        DriveConstants.SHOOT_ON_THE_MOVE_I,
        DriveConstants.SHOOT_ON_THE_MOVE_D);

    _rotationPID.setTolerance(DriveConstants.SHOOT_ON_THE_MOVE_TOLERANCE);
    _rotationPID.enableContinuousInput(-180.0, 180.0);

    _shotTimer = new Timer();
    _hasRunOnce = false;
    _triggerThreshold = 0.1;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _isFinished = false;
    _cmdIsRunning = true;
    // Cancel CmdAdjustShooterAutomatically while this command runs; reenable when it finishes.
    _shooter.setAutoShootEnabled(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_trigger.getAsDouble() > _triggerThreshold) {
      if (!_hasRunOnce) {
        _shotTimer.start();
        _hasRunOnce = true;
      }
    }


    // Get *translation only* of the robot
    _currentRobotTranslation = _drive.getPose().getTranslation();

    _speeds = _drive.getFieldRelativeChassisSpeeds();

    _timeUntilShot = ShooterConstants.TIME_TO_SHOOT - _shotTimer.get();

    if (_timeUntilShot < 0.0) {
      _timeUntilShot = 0.0;
    }

    // Calculate change in x and y distances due to time and velocity.
    _moveDelta = new Translation2d(_timeUntilShot * (_speeds.vxMetersPerSecond),
        _timeUntilShot * (_speeds.vyMetersPerSecond));

    // Add current position + change in position due to velocity to get future
    // position. This is where the robot will be at _timeUntilShot.
    _futureRobotTranslation = _currentRobotTranslation.plus(_moveDelta);

    // we add 180 because the intake is the front of the robot and we want the
    // shooter to face the speaker not the intake.
    _currentAngleRadians = _drive.getRotation().getRadians() + Math.PI;

    // Angle to the speaker from the future position as a Rotation2d.
    _futureAngleToSpeaker = _drive.getRotation2dToSpeaker(_futureRobotTranslation);

    // 
    _drive.setShootOnMoveGyro(_futureRobotTranslation.getAngle());

    // All PID calculations are done in radians, so convert our setpoint from a
    // Rotation2d to radians.
    _rotationPID.setSetpoint(_futureAngleToSpeaker.getRadians());


    // Angle in radians (omegaRadiansPerSecond) to pass to the drivetrain later in a
    // ChassisSpeeds object
    _correctedRotation = _rotationPID.calculate(
        (MathUtil.inputModulus(_currentAngleRadians, -1 * Math.PI, Math.PI)));

    // Get a Pose2d based on the newly-calculated future translation and angle.
    _futureRobotPose2d = new Pose2d(_futureRobotTranslation, _futureAngleToSpeaker);
    _correctedRadius = _drive.getRadiusToSpeakerInMeters(_futureRobotPose2d);

    // drive the robot based on the calculations from above
    _drive.runVelocity(
        _drive.transformJoystickInputsToChassisSpeeds(
          _translationXSupplier.getAsDouble(), 
          _translationYSupplier.getAsDouble(),
           _correctedRotation, false));
        

    if (_shooter.isAutoShootEnabled()) {
      if (_intake.hasNote()) {
        // if (_correctedZone != _shooter.getCurrentZone()) {
        //   // We want to shoot!
        //   _shooter.runShooterForZone(_correctedZone);
        // }
        _shooter.runShooterForRadius(_correctedRadius);

      } else {
        if (_shooter.getCurrentZone() != ShooterZone.Unknown) {
          _shooter.runShooterForZone(ShooterZone.Unknown);
        }
      }
    }

    // Adds a boolean that tells if this command is running to NetworkTables.
    Logger.recordOutput("Drive/shootOnTheMove/isRunning", _cmdIsRunning);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            _translationXSupplier.getAsDouble(),
            _translationYSupplier.getAsDouble(),
            0,
            _drive.getGyroscopeRotation()));
    _shotTimer.stop();
    _shotTimer.reset();
    _hasRunOnce = false;
    _cmdIsRunning = false;

    // Reenable CmdAdjustShooterAutomatically because this command is finished.
    _shooter.setAutoShootEnabled(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _isFinished;
  }
}

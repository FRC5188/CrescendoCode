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

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class CmdShootOnTheMove extends Command {
  /** Creates a new CmdDriveShootOnTheMove. */
  private final Drive _drive;
  private final Shooter _shooter;

  private boolean _isFinished;

  private Translation2d _currentRobotTranslation;
  private Rotation2d _currentAngleToSpeaker;

  private Translation2d _futureRobotTranslation;
  private Rotation2d _futureAngleToSpeaker;

  private ChassisSpeeds _speeds;
  private Rotation2d _correctedRotation;
  private double _timeUntilShot;
  private Translation2d _moveDelta;
  private DoubleSupplier _trigger;
  private Timer _shotTimer;
  private Boolean _hasRunOnce;
  private ShooterZone _correctedZone;
  private Pose2d _futureRobotPose2d;
  private double _triggerThreshold;

  /**
   * @param drivetrainSubsystem the drive subsystem
   * @param triggerAxis         button binding used
   **/
  public CmdShootOnTheMove(Drive drivetrainSubsystem,
      Shooter shooterSubsystem,
      DoubleSupplier triggerAxis) {

    _drive = drivetrainSubsystem;
    _shooter = shooterSubsystem;
    _trigger = triggerAxis;
    _shotTimer = new Timer();
    _hasRunOnce = false;
    _triggerThreshold = 0.01;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _isFinished = false;
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

    _currentAngleToSpeaker = _drive.getRotation2dToSpeaker(_currentRobotTranslation);

    _speeds = _drive.getFieldRelativeChassisSpeeds();

    _timeUntilShot = ShooterConstants.TIME_TO_SHOOT - _shotTimer.get();

    if (_timeUntilShot < 0.0) {
      _timeUntilShot = 0.0;
    }

    _moveDelta = new Translation2d(_timeUntilShot * (_speeds.vxMetersPerSecond),
        _timeUntilShot * (_speeds.vyMetersPerSecond));

    _futureRobotTranslation = _currentRobotTranslation.plus(_moveDelta);

    _futureAngleToSpeaker = _drive.getRotation2dToSpeaker(_futureRobotTranslation);

    // Calculate change in x and y distances due to time and velocity.
    _moveDelta = new Translation2d(_timeUntilShot * (_speeds.vxMetersPerSecond),
        _timeUntilShot * (_speeds.vyMetersPerSecond));

    // Add current position + change in position due to velocity to get future position.
    // This is where the robot will be at _timeUntilShot.
    _futureRobotTranslation = _currentRobotTranslation.plus(_moveDelta);

    // Angle to the speaker from the future position.
    _futureAngleToSpeaker = _drive.getRotation2dToSpeaker(_futureRobotTranslation);

    // Amount added to the current angle to the speaker to aim for the future.
    _correctedRotation = _futureAngleToSpeaker;

    // Get a Pose2d based on the newly-calculated future translation and angle.
    _futureRobotPose2d = new Pose2d(_futureRobotTranslation, _futureAngleToSpeaker); // TODO: Check! -KtH 2024-03-07

    _correctedZone = _shooter.getZoneFromRadius(_drive.getRadiusToSpeakerInMeters(_futureRobotPose2d));

    // this._drive.runVelocity( // TODO: Make it go worky
    // ChassisSpeeds.fromFieldRelativeSpeeds(
    
    // _translationYSupplier.getAsDouble(),
    // _correct,
    // _drive.getRotation()));

    // TODO: Add if statement like "isAutoShootEnabled"
    // TODO: Add shooting
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: Should we include?
    // this._drive.runVelocity(
    // ChassisSpeeds.fromFieldRelativeSpeeds(
    // _translationXSupplier.getAsDouble(),
    // _translationYSupplier.getAsDouble(),
    // 0,
    // _drive.getGyroscopeRotation()));
    _shotTimer.stop();
    _shotTimer.reset();
    _hasRunOnce = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _isFinished;
  }
}

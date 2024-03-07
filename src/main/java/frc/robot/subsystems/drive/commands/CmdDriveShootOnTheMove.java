// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class CmdDriveShootOnTheMove extends Command {
  /** Creates a new CmdDriveShootOnTheMove. */
  private final Drive _drive;

  private boolean _isFinished;

  private Translation2d _currentRobotTranslation;
  private Rotation2d _currentAngleToSpeaker;

  private Translation2d _futureRobotTranslation;
  private Rotation2d _futureAngleToSpeaker;

  private ChassisSpeeds _speeds;
  private Alliance _alliance;
  private double _correctionAngle;
  private double _timeUntilShot;
  private double _xDelta;
  private double _yDelta;
  private Translation2d _moveDelta;
  private Rotation2d _correctedPose;
  private DoubleSupplier _trigger;
  private Timer _shotTimer;
  private Boolean _hasRunOnce;
  private double _correctedZone;
  private Pose2d _futureRobotPose2d;
  private double _triggerThreshold;

  /**
   * @param drivetrainSubsystem the drive subsystem
   * @param triggerAxis         button binding used
   **/
  public CmdDriveShootOnTheMove(Drive drivetrainSubsystem,
      DoubleSupplier triggerAxis) {

    _drive = drivetrainSubsystem;
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

        _currentAngleToSpeaker = _drive.getRotation2dToSpeaker(_currentRobotTranslation);// TODO: CHECKKKK

        _speeds = _drive.getFieldRelativeChassisSpeeds();

        _timeUntilShot = ShooterConstants.TIME_TO_SHOOT - _shotTimer.get();
        
        if (_timeUntilShot < 0.0) {
          _timeUntilShot = 0.0;
        }

        _moveDelta = new Translation2d(_timeUntilShot * (_speeds.vxMetersPerSecond),
            _timeUntilShot * (_speeds.vyMetersPerSecond));

        _futureRobotTranslation = _currentRobotTranslation.plus(_moveDelta);

        _futureAngleToSpeaker = _drive.getRotation2dToSpeaker(_futureRobotTranslation);
        
        // this._drive.runVelocity(
        //         ChassisSpeeds.fromFieldRelativeSpeeds(
        //                 _translationXSupplier.getAsDouble(),
        //                 _translationYSupplier.getAsDouble(),
        //                 _rotationVal,
        //                 _drive.getRotation()));          
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // when we finish set the rotation to 0 but keep driving
    // this._drive.runVelocity(
    // ChassisSpeeds.fromFieldRelativeSpeeds(
    // _translationXSupplier.getAsDouble(),
    // _translationYSupplier.getAsDouble(),
    // 0,
    // _drive.getGyroscopeRotation()));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.shooter.ShooterIO;
import frc.robot.hardware.shooter.ShooterIOInputsAutoLogged;

public class Shooter extends SubsystemBase {
  public enum ShooterZone {
    // Here we define all of the zones for the shooter
    Subwoofer (0, 2.5, 45, 3000, 3000),
    Unknown (-1, -1, 0, 3000, 3000);

    private final double _lowBound;
    private final double _highBound;
    private final double _shooterAngle;
    private final double _leftFlywheelSpeed;
    private final double _rightFlywheelSpeed;

    ShooterZone(double lowBound, double highBound, double shooterAngle, double leftFlywheelSpeed, double rightFlywheelSpeed) {
        this._lowBound = lowBound;
        this._highBound = highBound;
        this._shooterAngle = shooterAngle;
        this._leftFlywheelSpeed = leftFlywheelSpeed;
        this._rightFlywheelSpeed = rightFlywheelSpeed;
    }

    // These functions can be called on an enum value to get various bits of data
    boolean radiusInZone(double radius) {
      return (radius >= _lowBound) || (radius < _highBound);
    }

    double get_shooterAngle() {
      return this._shooterAngle;
    }

    double get_leftFlywheelSpeed() {
      return this._leftFlywheelSpeed;
    }

    double get_rightFlywheelSpeed() {
      return this._rightFlywheelSpeed;
    }
  }

  private static boolean _autoShootEnabled = true;
  private final ShooterIO _shooterIO;
  private final ShooterIOInputsAutoLogged _shooterInputs = new ShooterIOInputsAutoLogged();
  private double _targetShooterPosition;

  private ShooterZone _currentShooterZone;

  public Shooter(ShooterIO shooterIO) {
    _shooterIO = shooterIO;
    _targetShooterPosition = ShooterConstants.SPEAKER_SCORE_ANGLE;
  }

  public void setTargetPosition(ShooterZone zone) {
    setTargetPositionAsAngle(zone.get_shooterAngle());
  }

  public void setTargetPositionAsAngle(double angle) {
    if (angle < ShooterConstants.MIN_SHOOTER_ANGLE) {
      // TODO: Log invalid angle: Parameter 'angle' must >= MIN_SHOOTER_ANGLE. -KtH
      // 2024/01/23
      return;

    } else if (angle > ShooterConstants.MAX_SHOOTER_ANGLE) {
      // TODO: Log invalid angle: Parameter 'angle' must <= MAX_SHOOTER_ANGLE. -KtH
      // 2024/01/23
      return;
    } else {
      _shooterIO.setTargetPositionAsDegrees(angle);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _shooterIO.updateInputs(_shooterInputs);
    Logger.processInputs("Drive/Gyro", _shooterInputs);
  }
  
  public double getCurrentPositionInDegrees() throws RuntimeException {
    double encoderValueAsRotations = _shooterInputs._angleEncoderPositionRotations;
    if (encoderValueAsRotations >= ShooterConstants.MAXIMUM_ANGLE_ENCODER_TURNS
        + Rotation2d.fromDegrees(10).getRotations()
        || encoderValueAsRotations <= ShooterConstants.MINIMUM_ANGLE_ENCODER_TURNS
            - Rotation2d.fromDegrees(10).getRotations()) {
      throw new RuntimeException(
          "It's impossible for the encoder to be this value. There must be a hardware error. Shut down this subsystem to not break everything.");
    } else {
      return Rotation2d.fromRotations(encoderValueAsRotations).getDegrees();
    }
  }

  public ShooterZone getZoneFromRadius(double radius) {
    for (ShooterZone zone : ShooterZone.values()) {
      if (zone.radiusInZone(radius)) {
        return zone;
      }
    }

    return ShooterZone.Unknown;
  }
  
  public boolean isAutoShootEnabled() {
    return _autoShootEnabled;
  }

  public void setAutoShootEnabled(boolean enabled) {
    _autoShootEnabled = enabled;
  }

  public void setShooterPosition(ShooterZone targetZone) {
    _currentShooterZone = targetZone;
    setTargetPositionAsAngle(targetZone.get_shooterAngle());
  }

  public boolean isReady(){
    boolean isFlywheelReady;
    boolean isAngleReady;
    // Here we'll check both the flywheel speed and the shooter angle. 
    // If both are within a certain range, we'll return true.

    // First, we'll check the flywheel speed with a deadband.
    isFlywheelReady = Math.abs(_shooterInputs._leftFlywheelMotorVelocityRotationsPerMin - _currentShooterZone.get_leftFlywheelSpeed()) < ShooterConstants.FLYWHEEL_DEADBAND
        && Math.abs(_shooterInputs._rightFlywheelMotorVelocityRotationsPerMin - _currentShooterZone.get_rightFlywheelSpeed()) < ShooterConstants.FLYWHEEL_DEADBAND;

    // Next, we'll check the shooter angle.
    isAngleReady = Math.abs(_shooterInputs._angleEncoderPositionRotations - _currentShooterZone.get_shooterAngle()) < ShooterConstants.ANGLE_ENCODER_DEADBAND_DEGREES;
  
    return isFlywheelReady && isAngleReady;
    // It should be noted that there is another method being worked on right now shooterInPosition() that eventually will be put into this. 
  }
  private boolean shooterInPosition() {
      return Math.abs(_targetShooterPosition - getCurrentPositionInDegrees()) <= ShooterConstants.ANGLE_ENCODER_DEADBAND_DEGREES;
  }
}

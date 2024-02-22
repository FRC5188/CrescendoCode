// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  public enum ShooterZone {
    Subwoofer(ShooterConstants.ZONE_SUBWOOFER_LOW_BOUND, 
              ShooterConstants.ZONE_SUBWOOFER_UPPER_BOUND,
              ShooterConstants.ZONE_SUBWOOFER_SHOOTER_ANGLE,
              ShooterConstants.ZONE_SUBWOOFER_FLYWHEEL_SPEED,
              ShooterConstants.ZONE_SUBWOOFER_FLYWHEEL_SPEED),
    Podium(ShooterConstants.ZONE_PODIUM_LOW_BOUND, 
              ShooterConstants.ZONE_PODIUM_UPPER_BOUND,
              ShooterConstants.ZONE_PODIUM_SHOOTER_ANGLE,
              ShooterConstants.ZONE_PODIUM_FLYWHEEL_SPEED,
              ShooterConstants.ZONE_PODIUM_FLYWHEEL_SPEED),
    Unknown(ShooterConstants.ZONE_UNKOWN_LOW_BOUND, 
              ShooterConstants.ZONE_UNKOWN_UPPER_BOUND,
              ShooterConstants.ZONE_UNKOWN_SHOOTER_ANGLE,
              ShooterConstants.ZONE_UNKOWN_FLYWHEEL_SPEED,
              ShooterConstants.ZONE_UNKOWN_FLYWHEEL_SPEED);

    private final double _lowBound;
    private final double _highBound;
    private final double _shooterAngle;
    private final double _leftFlywheelSpeed;
    private final double _rightFlywheelSpeed;

    ShooterZone(double lowBound, double highBound, double shooterAngle, double leftFlywheelSpeed,
        double rightFlywheelSpeed) {
      this._lowBound = lowBound;
      this._highBound = highBound;
      this._shooterAngle = shooterAngle;
      this._leftFlywheelSpeed = leftFlywheelSpeed;
      this._rightFlywheelSpeed = rightFlywheelSpeed;
    }

    // These functions can be called on an enum value to get various bits of data
    public boolean radiusInZone(double radius) {
      return (radius >= _lowBound) && (radius < _highBound);
    }

    public double getShooterAngle() {
      return this._shooterAngle;
    }

    public double getLeftFlywheelSpeed() {
      return this._leftFlywheelSpeed;
    }

    public double getRightFlywheelSpeed() {
      return this._rightFlywheelSpeed;
    }
  }

  private static boolean _autoShootEnabled = true;
  private final ShooterIO _shooterIO;
  private final ShooterIOInputsAutoLogged _shooterInputs = new ShooterIOInputsAutoLogged();
  private double _targetFlywheelSpeed = 0;
  private double _rightTargetFlywheelSpeed = 0;
  private double _targetShooterPosition;
  private PIDController _anglePid;

  private ShooterZone _currentShooterZone;

  private ShooterVisualizer _shooterVisualizer = new ShooterVisualizer();

  public Shooter(ShooterIO shooterIO) {
    _shooterIO = shooterIO;
    _currentShooterZone = ShooterZone.Unknown;
    _targetShooterPosition = _currentShooterZone.getShooterAngle();
    _anglePid = new PIDController(0.055, 0, 0.02);
    setTargetPositionAsAngle(_currentShooterZone.getShooterAngle());
  }

  public void runAnglePid() {
    _shooterIO.setAngleMotorSpeed(_anglePid.calculate(getCurrentPositionInDegrees()));
  }

  public void setTargetPosition(ShooterZone zone) {
    setTargetPositionAsAngle(zone.getShooterAngle());
  }

  public void setTargetPositionAsAngle(double angle) {
    if (angle < ShooterConstants.MINIMUM_ANGLE_ENCODER_ANGLE) {
      // TODO: Log invalid angle: Parameter 'angle' must >= MIN_SHOOTER_ANGLE. -KtH
      // 2024/01/23
      return;

    } else if (angle > ShooterConstants.MAXIMUM_ANGLE_ENCODER_ANGLE) {
      // TODO: Log invalid angle: Parameter 'angle' must <= MAX_SHOOTER_ANGLE. -KtH
      // 2024/01/23
      return;
    } else {
      //_shooterIO.setTargetPositionAsDegrees(angle);
      _anglePid.setSetpoint(angle);
    }
  }

  public double getCurrentPositionInDegrees() throws RuntimeException {
    if (_shooterInputs._angleEncoderPositionDegrees >= ShooterConstants.MAXIMUM_ANGLE_ENCODER_ANGLE
        + 10
        || _shooterInputs._angleEncoderPositionDegrees <= ShooterConstants.MINIMUM_ANGLE_ENCODER_ANGLE
            - 10) {
      throw new RuntimeException(
          "It's impossible for the encoder to be this value. There must be a hardware error. Shut down this subsystem to not break everything.");
    } else {
      return _shooterInputs._angleEncoderPositionDegrees;
    }
  }

  public ShooterZone getCurrentZone() {
    return _currentShooterZone;
  }

  private boolean areFlywheelsAtTargetSpeed() {
    return Math
        .abs(_shooterInputs._leftFlywheelMotorVelocityRotationsPerMin
            - _targetFlywheelSpeed) <= ShooterConstants.FLYWHEEL_SPEED_DEADBAND
        &&
        Math.abs(_shooterInputs._rightFlywheelMotorVelocityRotationsPerMin
            - _rightTargetFlywheelSpeed) <= ShooterConstants.FLYWHEEL_SPEED_DEADBAND;
  }

  public void runShooterForZone(ShooterZone zone) {
    setShooterPositionWithZone(zone);
    setFlywheelSpeed(zone._leftFlywheelSpeed);
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

  public void setShooterPositionWithZone(ShooterZone targetZone) {
    _currentShooterZone = targetZone;
    setTargetPositionAsAngle(targetZone.getShooterAngle());
  }

  private boolean shooterInPosition() {
    return Math.abs(_targetShooterPosition
        - getCurrentPositionInDegrees()) <= ShooterConstants.ANGLE_ENCODER_DEADBAND_DEGREES;
  }

  public void stopFlywheels() {
    _shooterIO.stopFlywheels();
  }

  public void setFlywheelSpeed(double speedInRPM) {
    this._targetFlywheelSpeed = speedInRPM;
    _shooterIO.setFlywheelSpeedRPM(speedInRPM);
  }

  public boolean isReady() {
    return shooterInPosition() && areFlywheelsAtTargetSpeed() && _currentShooterZone != ShooterZone.Unknown;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _shooterIO.updateInputs(_shooterInputs);
    Logger.processInputs("Shooter", _shooterInputs);

    // VISUALIZATION
    _shooterVisualizer.update(_shooterInputs._angleEncoderPositionDegrees);

    // LOGGING

    Logger.recordOutput("Shooter/Zone", _currentShooterZone.toString());
    Logger.recordOutput("Shooter/AngleDesiredDegrees", _currentShooterZone.getShooterAngle());
    Logger.recordOutput("Shooter/FlywheelSetpoint", this._targetFlywheelSpeed);
    // Logger.recordOutput("Mechanism2D/Shooter", _shooterVisualizer.getMechanism());
  }
}

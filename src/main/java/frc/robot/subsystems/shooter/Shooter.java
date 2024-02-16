// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.shooter.ShooterIO;
import frc.robot.hardware.shooter.ShooterIOInputsAutoLogged;

public class Shooter extends SubsystemBase {
  public enum ShooterZone {
    // Here we define all of the zones for the shooter
    Subwoofer(0, 2.5, 45, 3000, 3000),
    Unknown(-1, -1, 0, 3000, 3000);

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

    /**
     * Returns current shooter angle
     * @return
     */
    public double getShooterAngle() {
      return this._shooterAngle;
    }

    /**
     * Returns current speed of Left Flywheel
     * @return
     */
    public double getLeftFlywheelSpeed() {
      return this._leftFlywheelSpeed;
    }

    /**
     * Returns current speed of Right Flywheel
     * @return
     */
    public double getRightFlywheelSpeed() {
      return this._rightFlywheelSpeed;
    }
  }

  private static boolean _autoShootEnabled = true;
  private final ShooterIO _shooterIO;
  private final ShooterIOInputsAutoLogged _shooterInputs = new ShooterIOInputsAutoLogged();
  private double _leftTargetFlywheelSpeed = 0;
  private double _rightTargetFlywheelSpeed = 0;
  private double _targetShooterPosition;
  private PIDController _anglePid;

  private ShooterZone _currentShooterZone;

  private ShooterVisualizer _shooterVisualizer = new ShooterVisualizer();

  /**
   * Initializes shooter
   * @param shooterIO
   */
  public Shooter(ShooterIO shooterIO) {
    _shooterIO = shooterIO;
    _targetShooterPosition = getCurrentPositionInDegrees();
    _anglePid = new PIDController(0.055, 0, 0);
    setTargetPositionAsAngle(15);
  }

  /**
   * Initializes angle motor
   */
  public void runAnglePid() {
    _shooterIO.setAngleMotorSpeed(-_anglePid.calculate(getCurrentPositionInDegrees()));
  }

  /**
   * Sets the target position to correspond to the desired zone
   * @param zone
   */
  public void setTargetPosition(ShooterZone zone) {
    setTargetPositionAsAngle(zone.getShooterAngle());
  }

  /**
   * Ensures that desired shooter angle is possible to achieve
   * @param angle
   */
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

  /**
   * Throws RuntimeException if _angleEncoderPositionDegrees <= MINIMUM_ANGLE_ENCODER_ANGLE - 10
   * @return
   * @throws RuntimeException
   */
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

  /**
   * Returns what the occupied shooter zone currently is
   * @return
   */
  public ShooterZone getCurrentZone() {
    return _currentShooterZone;
  }

  /**
   * Checks if the flywheels are at the target speed in any direction
   * @return
   */
  private boolean areFlywheelsAtTargetSpeed() {
    return Math
        .abs(_shooterInputs._leftFlywheelMotorVelocityRotationsPerMin
            - _leftTargetFlywheelSpeed) <= ShooterConstants.FLYWHEEL_SPEED_DEADBAND
        &&
        Math.abs(_shooterInputs._rightFlywheelMotorVelocityRotationsPerMin
            - _rightTargetFlywheelSpeed) <= ShooterConstants.FLYWHEEL_SPEED_DEADBAND;
  }

  /**
   * Sets the shooter position and flywheel speeds according to the current shooter zone
   * @param zone
   */
  public void runShooterForZone(ShooterZone zone) {
    setShooterPositionWithZone(zone);
    setLeftFlywheelSpeed(zone._leftFlywheelSpeed);
    setRightFlywheelSpeed(zone._rightFlywheelSpeed);
  }

  /**
   * Gets zone from radius
   * @param radius
   * @return
   */
  public ShooterZone getZoneFromRadius(double radius) {
    for (ShooterZone zone : ShooterZone.values()) {
      if (zone.radiusInZone(radius)) {
        return zone;
      }
    }

    return ShooterZone.Unknown;
  }

  /**
   * Returns whether the autoshooter is enabled
   * @return
   */
  public boolean isAutoShootEnabled() {
    return _autoShootEnabled;
  }

  /**
   * Enables autoshoot
   * @param enabled
   */
  public void setAutoShootEnabled(boolean enabled) {
    _autoShootEnabled = enabled;
  }

  /**
   * Sets the angle of the shooter depending on what zone the robot is in
   * @param targetZone
   */
  public void setShooterPositionWithZone(ShooterZone targetZone) {
    _currentShooterZone = targetZone;
      setTargetPositionAsAngle(targetZone.getShooterAngle());
  }

  /**
   * Checks if the shooter is in the right position
   * @return
   */
  private boolean shooterInPosition() {
    return Math.abs(_targetShooterPosition
        - getCurrentPositionInDegrees()) <= ShooterConstants.ANGLE_ENCODER_DEADBAND_DEGREES;
  }

  /**
   * Stops flywheels
   */
  public void stopFlywheels() {
    _shooterIO.stopFlywheels();
  }

  /**
   * Tells the left flywheel motor how fast to spin the flywheel
   * @param speedInRPM
   */
  public void setLeftFlywheelSpeed(double speedInRPM) {
    _shooterIO.setLeftFlywheelSpeedRPM(speedInRPM);
  }

  /**
   * Tells the right flywheel motor how fast to spin the flywheel
   * @param speedInRPM
   */
  public void setRightFlywheelSpeed(double speedInRPM) {
    _shooterIO.setRightFlywheelSpeedRPM(speedInRPM);
  }

  /**
   * Checks if the shooter is at the right angle and if the flywheels are at the right speeds and if the robot is in a known zone
   * If all three criteria return true then it's ready to shoot
   * @return
   */
  public boolean isReady() {
    return shooterInPosition() && areFlywheelsAtTargetSpeed() && _currentShooterZone != ShooterZone.Unknown;
  }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _shooterIO.updateInputs(_shooterInputs);
    Logger.processInputs("Shooter", _shooterInputs);
    double angle = _shooterInputs._angleEncoderPositionDegrees;
    _shooterVisualizer.update(angle);
    Logger.recordOutput("Shooter/AngleDegrees", angle);
    Logger.recordOutput("Mechanism2D/Shooter", _shooterVisualizer.getMechanism());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  public enum ShooterZone {
    Subwoofer(ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.LOW_BOUND_TUNABLE.get(), // In the future we might want to not pull this every cycle though we'll see :) -MG
              ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.UPPER_BOUND_TUNABLE.get(),
              ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.SHOOTER_ANGLE_TUNABLE.get(),
              ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.FLYWHEEL_SPEED_TUNABLE.get(),
              ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.FLYWHEEL_SPEED_TUNABLE.get()),
    Podium(ShooterConstants.ZONE_CONSTANTS.PODIUM.LOW_BOUND_TUNABLE.get(),
              ShooterConstants.ZONE_CONSTANTS.PODIUM.UPPER_BOUND_TUNABLE.get(),
              ShooterConstants.ZONE_CONSTANTS.PODIUM.SHOOTER_ANGLE_TUNABLE.get(),
              ShooterConstants.ZONE_CONSTANTS.PODIUM.FLYWHEEL_SPEED_TUNABLE.get(),
              ShooterConstants.ZONE_CONSTANTS.PODIUM.FLYWHEEL_SPEED_TUNABLE.get()),
    Unknown(ShooterConstants.ZONE_CONSTANTS.UNKNOWN.LOW_BOUND_TUNABLE.get(), 
              ShooterConstants.ZONE_CONSTANTS.UNKNOWN.UPPER_BOUND_TUNABLE.get(),
              ShooterConstants.ZONE_CONSTANTS.UNKNOWN.SHOOTER_ANGLE_TUNABLE.get(),
              ShooterConstants.ZONE_CONSTANTS.UNKNOWN.FLYWHEEL_SPEED_TUNABLE.get(),
              ShooterConstants.ZONE_CONSTANTS.UNKNOWN.FLYWHEEL_SPEED_TUNABLE.get());

    private final double _lowBound;
    private final double _highBound;
    private double _shooterAngle;
    private double _leftFlywheelSpeed;
    private double _rightFlywheelSpeed;

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

  private static boolean _autoShootEnabled = false; // This should default to disabled and then enabled later.

  private final ShooterIO _shooterIO;
  private final ShooterIOInputsAutoLogged _shooterInputs = new ShooterIOInputsAutoLogged();

  private PIDController _anglePid;

  private ShooterZone _currentShooterZone;
  private double _targetShooterPosition;
  private double _targetFlywheelSpeed;

  private ShooterVisualizer _shooterVisualizer = new ShooterVisualizer();

  public Shooter(ShooterIO shooterIO) {
    _shooterIO = shooterIO;
    _currentShooterZone = ShooterZone.Unknown;
    _targetShooterPosition = _currentShooterZone.getShooterAngle();
    _anglePid = new PIDController(
      ShooterConstants.PID.ANGLE.KP,
      ShooterConstants.PID.ANGLE.KI,
      ShooterConstants.PID.ANGLE.KD
    );
    setTargetPositionAsAngle(_currentShooterZone.getShooterAngle());
  }

  public void runAnglePid() {
    _shooterIO.setAngleMotorSpeed(_anglePid.calculate(getCurrentPositionInDegrees()));
  }

  public void setTargetPosition(ShooterZone zone) {
    this._currentShooterZone = zone;
    setTargetPositionAsAngle(zone.getShooterAngle());
  }

  public void setTargetPositionAsAngle(double angle) {
    if (angle < ShooterConstants.MINIMUM_ANGLE_ENCODER_ANGLE) {
      System.out.println("Invalid angle: Parameter 'angle' must >= MIN_SHOOTER_ANGLE.");
      return;
    } else if (angle > ShooterConstants.MAXIMUM_ANGLE_ENCODER_ANGLE) {
      System.out.println("Invalid angle: Parameter 'angle' must <= MAX_SHOOTER_ANGLE.");
      return;
    } else {
      this._targetShooterPosition = angle;
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
    return this._currentShooterZone;
  }


  private boolean areFlywheelsAtTargetSpeed() {
    return Math
        .abs(_shooterInputs._leftFlywheelMotorVelocityRotationsPerMin
            - this._targetFlywheelSpeed) <= ShooterConstants.SOFTWARE.FLYWHEEL_SPEED_DEADBAND
        &&
        Math.abs(_shooterInputs._rightFlywheelMotorVelocityRotationsPerMin
            - this._targetFlywheelSpeed) <= ShooterConstants.SOFTWARE.FLYWHEEL_SPEED_DEADBAND;
  }

  public void runShooterForZone(ShooterZone zone) {
    setShooterPositionWithZone(zone);
    setFlywheelSpeed(zone._leftFlywheelSpeed); // ODO: If we're going to do this then I don't see the benefit of having speeds for two different speeds for motors.
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
    this._currentShooterZone = targetZone;
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

    // Here is where we'll poll whether the constants in shooter have changed.
    // If they have, we'll update them.

    pollForTunableChanges();

    // VISUALIZATION
    _shooterVisualizer.update(_shooterInputs._angleEncoderPositionDegrees);

    // LOGGING

    Logger.recordOutput("Shooter/Zone", _currentShooterZone.toString());
    Logger.recordOutput("Shooter/AngleDesiredDegrees", _currentShooterZone.getShooterAngle());
    Logger.recordOutput("Shooter/FlywheelSetpoint", this._targetFlywheelSpeed);
    // Logger.recordOutput("Mechanism2D/Shooter", _shooterVisualizer.getMechanism());
  }

  private void pollForTunableChanges() {
    // Poll for changes made to tunable constants and if there is a change update the constant.

    // ================ ANGLE MOTOR PID ================ //

    if (ShooterConstants.PID.ANGLE.KP_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.PID.ANGLE.KP = ShooterConstants.PID.ANGLE.KP_TUNABLE.get();
      _anglePid.setP(ShooterConstants.PID.ANGLE.KP);
    }

    if (ShooterConstants.PID.ANGLE.KI_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.PID.ANGLE.KI = ShooterConstants.PID.ANGLE.KI_TUNABLE.get();
      _anglePid.setI(ShooterConstants.PID.ANGLE.KI);
    }

    if (ShooterConstants.PID.ANGLE.KD_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.PID.ANGLE.KD = ShooterConstants.PID.ANGLE.KD_TUNABLE.get();
      _anglePid.setD(ShooterConstants.PID.ANGLE.KD);
    }

    if (ShooterConstants.PID.ANGLE.KF_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.PID.ANGLE.KF = ShooterConstants.PID.ANGLE.KF_TUNABLE.get();
      // No feedforward implementation right now. Added in case we want to add it later.
    }

    // ================ FLYWHEEL PID ================ //

    if (ShooterConstants.PID.FLYHWEEL.KP_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.PID.FLYHWEEL.KP = ShooterConstants.PID.FLYHWEEL.KP_TUNABLE.get();
      _shooterIO.configFlywheelPIDs(ShooterConstants.PID.FLYHWEEL.KP, ShooterConstants.PID.FLYHWEEL.KI,
          ShooterConstants.PID.FLYHWEEL.KD, ShooterConstants.PID.FLYHWEEL.KF);
    }

    if (ShooterConstants.PID.FLYHWEEL.KI_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.PID.FLYHWEEL.KI = ShooterConstants.PID.FLYHWEEL.KI_TUNABLE.get();
      _shooterIO.configFlywheelPIDs(ShooterConstants.PID.FLYHWEEL.KP, ShooterConstants.PID.FLYHWEEL.KI,
          ShooterConstants.PID.FLYHWEEL.KD, ShooterConstants.PID.FLYHWEEL.KF);
    }

    if (ShooterConstants.PID.FLYHWEEL.KD_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.PID.FLYHWEEL.KD = ShooterConstants.PID.FLYHWEEL.KD_TUNABLE.get();
      _shooterIO.configFlywheelPIDs(ShooterConstants.PID.FLYHWEEL.KP, ShooterConstants.PID.FLYHWEEL.KI,
          ShooterConstants.PID.FLYHWEEL.KD, ShooterConstants.PID.FLYHWEEL.KF);
    }

    if (ShooterConstants.PID.FLYHWEEL.KF_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.PID.FLYHWEEL.KF = ShooterConstants.PID.FLYHWEEL.KF_TUNABLE.get();
      _shooterIO.configFlywheelPIDs(ShooterConstants.PID.FLYHWEEL.KP, ShooterConstants.PID.FLYHWEEL.KI,
          ShooterConstants.PID.FLYHWEEL.KD, ShooterConstants.PID.FLYHWEEL.KF);
    }

    // ================ ZONE CONSTANTS ================ //

    if (ShooterConstants.SOFTWARE.FLYWHEEL_SPEED_DEADBAND_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.SOFTWARE.FLYWHEEL_SPEED_DEADBAND = ShooterConstants.SOFTWARE.FLYWHEEL_SPEED_DEADBAND_TUNABLE.get();
    }

    if (ShooterConstants.ZONE_CONSTANTS.PODIUM.FLYWHEEL_SPEED_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.ZONE_CONSTANTS.PODIUM.FLYWHEEL_SPEED = ShooterConstants.ZONE_CONSTANTS.PODIUM.FLYWHEEL_SPEED_TUNABLE.get();
    }

    if (ShooterConstants.ZONE_CONSTANTS.PODIUM.LOW_BOUND_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.ZONE_CONSTANTS.PODIUM.LOW_BOUND = ShooterConstants.ZONE_CONSTANTS.PODIUM.LOW_BOUND_TUNABLE.get();
    }

    if (ShooterConstants.ZONE_CONSTANTS.PODIUM.UPPER_BOUND_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.ZONE_CONSTANTS.PODIUM.UPPER_BOUND = ShooterConstants.ZONE_CONSTANTS.PODIUM.UPPER_BOUND_TUNABLE.get();
    }

    if (ShooterConstants.ZONE_CONSTANTS.PODIUM.SHOOTER_ANGLE_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.ZONE_CONSTANTS.PODIUM.SHOOTER_ANGLE = ShooterConstants.ZONE_CONSTANTS.PODIUM.SHOOTER_ANGLE_TUNABLE.get();
    }

    if (ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.FLYWHEEL_SPEED_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.FLYWHEEL_SPEED = ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.FLYWHEEL_SPEED_TUNABLE.get();
    }

    if (ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.LOW_BOUND_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.LOW_BOUND = ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.LOW_BOUND_TUNABLE.get();
    }

    if (ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.UPPER_BOUND_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.UPPER_BOUND = ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.UPPER_BOUND_TUNABLE.get();
    }

    if (ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.SHOOTER_ANGLE_TUNABLE.hasChanged(hashCode())) {
      ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.SHOOTER_ANGLE = ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.SHOOTER_ANGLE_TUNABLE.get();
    }
  }
}

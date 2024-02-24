// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.tunable.LoggedTunableNumber;
import frc.robot.util.tunable.Tunable;

public class Shooter extends SubsystemBase implements Tunable {
  public enum ShooterZone {
    Subwoofer(ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.LOW_BOUND, 
              ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.UPPER_BOUND,
              ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.SHOOTER_ANGLE,
              ShooterConstants.ZONE_CONSTANTS.SUBWOOFER.FLYWHEEL_SPEED),
    Podium(ShooterConstants.ZONE_CONSTANTS.PODIUM.LOW_BOUND,
              ShooterConstants.ZONE_CONSTANTS.PODIUM.UPPER_BOUND,
              ShooterConstants.ZONE_CONSTANTS.PODIUM.SHOOTER_ANGLE,
              ShooterConstants.ZONE_CONSTANTS.PODIUM.FLYWHEEL_SPEED),
    Unknown(ShooterConstants.ZONE_CONSTANTS.UNKNOWN.LOW_BOUND, 
              ShooterConstants.ZONE_CONSTANTS.UNKNOWN.UPPER_BOUND,
              ShooterConstants.ZONE_CONSTANTS.UNKNOWN.SHOOTER_ANGLE,
              ShooterConstants.ZONE_CONSTANTS.UNKNOWN.FLYWHEEL_SPEED);

    private final LoggedTunableNumber _lowBound;
    private final LoggedTunableNumber _highBound;
    private LoggedTunableNumber _shooterAngle;
    private LoggedTunableNumber _flywheelSpeed;

    ShooterZone(LoggedTunableNumber lowBound, LoggedTunableNumber highBound, LoggedTunableNumber shooterAngle, LoggedTunableNumber flywheelSpeed) {
      this._lowBound = lowBound;
      this._highBound = highBound;
      this._shooterAngle = shooterAngle;
      this._flywheelSpeed = flywheelSpeed;
    }

    // These functions can be called on an enum value to get various bits of data
    public boolean radiusInZone(double radius) {
      return (radius >= _lowBound.get()) && (radius < _highBound.get());
    }

    public double getShooterAngle() {
      return this._shooterAngle.get();
    }

    public double getFlywheelSpeed() {
      return this._flywheelSpeed.get();
    }
  }

  private static boolean _autoShootEnabled = false; // This should default to disabled and then enabled later.

  private final ShooterIO _shooterIO;
  private final ShooterIOInputsAutoLogged _shooterInputs = new ShooterIOInputsAutoLogged();

  private PIDController _anglePid;

  private ShooterZone _currentShooterZone;

  private ShooterVisualizer _shooterVisualizer = new ShooterVisualizer();

  public Shooter(ShooterIO shooterIO) {
    _shooterIO = shooterIO;

    _currentShooterZone = ShooterZone.Unknown;

    // This should be updated later in the poll() method.
    _anglePid = new PIDController(
      ShooterConstants.PID.ANGLE.KP.get(),
      ShooterConstants.PID.ANGLE.KI.get(),
      ShooterConstants.PID.ANGLE.KD.get()
    );

    // Updates the PID with the starting configuration of the robot.
    this.setTargetPositionAsAngle(this._currentShooterZone.getShooterAngle());
    this.setFlywheelSpeed(this._currentShooterZone.getFlywheelSpeed());
  }

  public void runAnglePid() {
    _shooterIO.setAngleMotorSpeed(_anglePid.calculate(getCurrentPositionInDegrees()));
  }

  public void setTargetPosition(ShooterZone zone) {
    this._currentShooterZone = zone;
    this.setTargetPositionAsAngle(this._currentShooterZone.getShooterAngle());
    this.setFlywheelSpeed(this._currentShooterZone.getFlywheelSpeed());
  }

  public void setTargetPositionAsAngle(double angle) {
    if (angle < ShooterConstants.MINIMUM_ANGLE_ENCODER_ANGLE) {
      System.out.println("Invalid angle: Parameter 'angle' must >= MIN_SHOOTER_ANGLE.");
      return;
    } else if (angle > ShooterConstants.MAXIMUM_ANGLE_ENCODER_ANGLE) {
      System.out.println("Invalid angle: Parameter 'angle' must <= MAX_SHOOTER_ANGLE.");
      return;
    } else {
      _anglePid.setSetpoint(angle);
    }
  }

  public double getCurrentPositionInDegrees() throws RuntimeException {
    if (_shooterInputs._angleEncoderPositionDegrees >= ShooterConstants.MAXIMUM_ANGLE_ENCODER_ANGLE
        + 10 // Since there are natural fluxuations that might be above our maximum we'll add a buffer though if we go over this then something has gone wrong.
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
            - this._currentShooterZone.getFlywheelSpeed()) <= ShooterConstants.SOFTWARE.FLYWHEEL_SPEED_DEADBAND.get()
        &&
        Math.abs(_shooterInputs._rightFlywheelMotorVelocityRotationsPerMin
            - this._currentShooterZone.getFlywheelSpeed()) <= ShooterConstants.SOFTWARE.FLYWHEEL_SPEED_DEADBAND.get();
  }

  public void runShooterForZone(ShooterZone zone) {
    setShooterPositionWithZone(zone);
    setFlywheelSpeed(zone.getFlywheelSpeed());
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
    return Math.abs(this._currentShooterZone.getShooterAngle()
        - getCurrentPositionInDegrees()) <= ShooterConstants.ANGLE_ENCODER_DEADBAND_DEGREES;
  }

  public void stopFlywheels() {
    _shooterIO.stopFlywheels();
  }

  public void setFlywheelSpeed(double speedInRPM) {
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
    Logger.recordOutput("Shooter/FlywheelSetpoint", this._currentShooterZone.getFlywheelSpeed());
    //Logger.recordOutput("Mechanism2D/Shooter", _shooterVisualizer.getMechanism());


  }

  @Override
  public void poll() {
    // Poll for changes made to tunable constants and if there is a change update the constant.

    // ================ ANGLE MOTOR PID ================ //

    if (ShooterConstants.PID.ANGLE.KP.hasChanged(hashCode())) {
      _anglePid.setP(ShooterConstants.PID.ANGLE.KP.get());
    }

    if (ShooterConstants.PID.ANGLE.KI.hasChanged(hashCode())) {
      _anglePid.setI(ShooterConstants.PID.ANGLE.KI.get());
    }

    if (ShooterConstants.PID.ANGLE.KD.hasChanged(hashCode())) {
      _anglePid.setD(ShooterConstants.PID.ANGLE.KD.get());
    }

    if (ShooterConstants.PID.ANGLE.KF.hasChanged(hashCode())) {
      // No feedforward implementation right now. Added in case we want to add it later.
    }

    // ================ FLYWHEEL PID ================ //

    if (ShooterConstants.PID.FLYHWEEL.KP.hasChanged(hashCode())) {
      _shooterIO.configFlywheelPIDs(ShooterConstants.PID.FLYHWEEL.KP.get(), ShooterConstants.PID.FLYHWEEL.KI.get(),
          ShooterConstants.PID.FLYHWEEL.KD.get(), ShooterConstants.PID.FLYHWEEL.KF.get());
    }

    if (ShooterConstants.PID.FLYHWEEL.KI.hasChanged(hashCode())) {
      _shooterIO.configFlywheelPIDs(ShooterConstants.PID.FLYHWEEL.KP.get(), ShooterConstants.PID.FLYHWEEL.KI.get(),
          ShooterConstants.PID.FLYHWEEL.KD.get(), ShooterConstants.PID.FLYHWEEL.KF.get());
    }

    if (ShooterConstants.PID.FLYHWEEL.KD.hasChanged(hashCode())) {
      _shooterIO.configFlywheelPIDs(ShooterConstants.PID.FLYHWEEL.KP.get(), ShooterConstants.PID.FLYHWEEL.KI.get(),
          ShooterConstants.PID.FLYHWEEL.KD.get(), ShooterConstants.PID.FLYHWEEL.KF.get());
    }

    if (ShooterConstants.PID.FLYHWEEL.KF.hasChanged(hashCode())) {
      _shooterIO.configFlywheelPIDs(ShooterConstants.PID.FLYHWEEL.KP.get(), ShooterConstants.PID.FLYHWEEL.KI.get(),
          ShooterConstants.PID.FLYHWEEL.KD.get(), ShooterConstants.PID.FLYHWEEL.KF.get());
    }
  }
}

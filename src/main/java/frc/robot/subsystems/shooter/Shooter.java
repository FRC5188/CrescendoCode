// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
  public enum ShooterZone {
    Subwoofer(ShooterConstants.ZONE.ZONE_SUBWOOFER_LOW_BOUND, 
              ShooterConstants.ZONE.ZONE_SUBWOOFER_UPPER_BOUND,
              ShooterConstants.ZONE.ZONE_SUBWOOFER_SHOOTER_ANGLE,
              ShooterConstants.ZONE.ZONE_SUBWOOFER_FLYWHEEL_SPEED,
              ShooterConstants.ZONE.ZONE_SUBWOOFER_FLYWHEEL_SPEED),
    Podium(ShooterConstants.ZONE.ZONE_PODIUM_LOW_BOUND, 
              ShooterConstants.ZONE.ZONE_PODIUM_UPPER_BOUND,
              ShooterConstants.ZONE.ZONE_PODIUM_SHOOTER_ANGLE,
              ShooterConstants.ZONE.ZONE_PODIUM_FLYWHEEL_SPEED,
              ShooterConstants.ZONE.ZONE_PODIUM_FLYWHEEL_SPEED),
    Unknown(ShooterConstants.ZONE.ZONE_UNKNOWN_LOW_BOUND, 
              ShooterConstants.ZONE.ZONE_UNKNOWN_UPPER_BOUND,
              ShooterConstants.ZONE.ZONE_UNKNOWN_SHOOTER_ANGLE,
              ShooterConstants.ZONE.ZONE_UNKNOWN_FLYWHEEL_SPEED,
              ShooterConstants.ZONE.ZONE_UNKNOWN_FLYWHEEL_SPEED);

    private final LoggedTunableNumber _lowBound;
    private final LoggedTunableNumber _highBound;
    private final LoggedTunableNumber _shooterAngle;
    private final LoggedTunableNumber _leftFlywheelSpeed;
    private final LoggedTunableNumber _rightFlywheelSpeed;

    ShooterZone(LoggedTunableNumber lowBound, 
                LoggedTunableNumber highBound, 
                LoggedTunableNumber shooterAngle, 
                LoggedTunableNumber leftFlywheelSpeed,
                LoggedTunableNumber rightFlywheelSpeed) {
      this._lowBound = lowBound;
      this._highBound = highBound;
      this._shooterAngle = shooterAngle;
      this._leftFlywheelSpeed = leftFlywheelSpeed;
      this._rightFlywheelSpeed = rightFlywheelSpeed;
    }

    // These functions can be called on an enum value to get various bits of data
    public boolean radiusInZone(double radius) {
      return (radius >= _lowBound.get()) && (radius < _highBound.get());
    }

    public double getShooterAngle() {
      return this._shooterAngle.get();
    }

    public double getLeftFlywheelSpeed() {
      return this._leftFlywheelSpeed.get();
    }

    public double getRightFlywheelSpeed() {
      return this._rightFlywheelSpeed.get();
    }
  }

  private static boolean _autoShootEnabled = true;
  private final ShooterIO _shooterIO;
  private final ShooterIOInputsAutoLogged _shooterInputs = new ShooterIOInputsAutoLogged();
  private double _targetFlywheelSpeed = 0;
  private double _rightTargetFlywheelSpeed = 0;
  private double _targetShooterPositionAngle;
  private PIDController _anglePid;
  private ShooterZone _currentShooterZone;
  private ShooterVisualizer _shooterVisualizer = new ShooterVisualizer();

  public Shooter(ShooterIO shooterIO) {
    _shooterIO = shooterIO;
    _currentShooterZone = ShooterZone.Unknown;
    _anglePid = new PIDController(
      ShooterConstants.PID.PIVOT.KP.get(), 
      ShooterConstants.PID.PIVOT.KI.get(),
      ShooterConstants.PID.PIVOT.KD.get());

    setTargetPositionAsAngle(_currentShooterZone.getShooterAngle());
  }

  public void setCurrentZone(ShooterZone zone){
    _currentShooterZone = zone;
  }

  public void runAnglePid() {
    _shooterIO.setAngleMotorSpeed(_anglePid.calculate(getCurrentPositionInDegrees()));
  }

  public void setTargetPosition(ShooterZone zone) {
    setTargetPositionAsAngle(zone.getShooterAngle());
  }

  public void setTargetPositionAsAngle(double angle) {
    if (angle < ShooterConstants.ENCODER.MINIMUM_ANGLE_ENCODER_ANGLE) {
      System.err.println("Invalid angle: Parameter 'angle' must >= MIN_SHOOTER_ANGLE.");
      return;

    } else if (angle > ShooterConstants.ENCODER.MAXIMUM_ANGLE_ENCODER_ANGLE) {
      System.err.println("Invalid angle: Parameter 'angle' must <= MAX_SHOOTER_ANGLE.");
      return;
    } else {
      _anglePid.setSetpoint(angle);
      _targetShooterPositionAngle = angle;
    }
  }

  /**
   * Get the desired position of the shooter in degrees.
   * This is the same as the setpoint of the PID.
   * 
   * @return The target position of the shooter
   */
  public double getTargetPositionAsAngle(){
    return this._targetShooterPositionAngle;
  }

  /**
   * The current angle of the shooter in degrees. This value is determined directly
   * from the rev encoder on the pivot shaft.
   * 
   * @return shooter angle in degrees
   */
  public double getCurrentPositionInDegrees(){
    // TODO: log an error but do not throw an exception. Not right now at least. Maybe later
    // Garrett 2/26/24

    // if (_shooterInputs._angleEncoderPositionDegrees >= ShooterConstants.MAXIMUM_ANGLE_ENCODER_ANGLE
    //     + 10
    //     || _shooterInputs._angleEncoderPositionDegrees <= ShooterConstants.MINIMUM_ANGLE_ENCODER_ANGLE
    //         - 10) {
    // } else {
      return _shooterInputs._angleEncoderPositionDegrees;
    // }
  }

  /**
   * The current zone the shooter thinks it is in.
   * 
   * @return Current Zone
   */
  public ShooterZone getCurrentZone() {
    return _currentShooterZone;
  }

  // TODO: THIS SHOULD BE SET AS MATH.ABS() ONCE SHOOTER FLYWHEEL PIDS ARE FIXED
  private boolean areFlywheelsAtTargetSpeed() {
    return _targetFlywheelSpeed - _shooterInputs._leftFlywheelMotorVelocityRotationsPerMin <= ShooterConstants.PID.FLYHWEELS.TOLERANCE
        &&
            _rightTargetFlywheelSpeed - _shooterInputs._rightFlywheelMotorVelocityRotationsPerMin <= ShooterConstants.PID.FLYHWEELS.TOLERANCE;
  }

  /**
   * Set the shooter position for a given zone and also set the flywheel speed for that zone
   * 
   * @param zone
   */
  public void runShooterForZone(ShooterZone zone) {
    setShooterPositionWithZone(zone);
    setFlywheelSpeed(zone._leftFlywheelSpeed.get());
  }

  /**
   * Figure out what zone matches a given radius from the speaker.
   * Returns the unkown zone if no zone matches.
   * 
   * @param radius
   * @return the zone used for that radius
   */
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

  /**
   * Set the position angle of the shooter based on a zone.
   * 
   * @param targetZone
   */
  public void setShooterPositionWithZone(ShooterZone targetZone) {
    _currentShooterZone = targetZone;
    setTargetPositionAsAngle(targetZone.getShooterAngle());
  }

  private boolean shooterInPosition() {
    //System.out.printf("target shooter angle: %f, current shooter angle: %f\n", _targetShooterPositionAngle, getCurrentPositionInDegrees());
    return Math.abs(_targetShooterPositionAngle
        - getCurrentPositionInDegrees()) <= ShooterConstants.ENCODER.ANGLE_ENCODER_DEADBAND_DEGREES;
  }

  /**
   * Set the fly wheel speed to zero.
   */
  public void stopFlywheels() {
    _shooterIO.stopFlywheels();
  }

  public void setFlywheelSpeed(double speedInRPM) {
    this._targetFlywheelSpeed = speedInRPM;
    _shooterIO.setFlywheelSpeedRPM(speedInRPM);
  }

  public boolean isReady() {
    //System.out.printf("Shooter in Position: %b,\n speed: %f, target speed: %f \n current shooter zone: ", shooterInPosition(), _shooterInputs._leftFlywheelMotorVelocityRotationsPerMin, _targetFlywheelSpeed);
    System.out.println(_currentShooterZone);
    return shooterInPosition() && areFlywheelsAtTargetSpeed() && _currentShooterZone != ShooterZone.Unknown;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _shooterIO.updateInputs(_shooterInputs);
    Logger.processInputs("Shooter", _shooterInputs);

    // VISUALIZATION
    _shooterVisualizer.update(_shooterInputs._angleEncoderPositionDegrees);

    // TUNABLES FOR PIVOT PID
    if (ShooterConstants.PID.PIVOT.KP.hasChanged(hashCode())) {
      _anglePid.setP(ShooterConstants.PID.PIVOT.KP.get());
    }

    if (ShooterConstants.PID.PIVOT.KI.hasChanged(hashCode())) {
      _anglePid.setI(ShooterConstants.PID.PIVOT.KI.get());
    }

    if (ShooterConstants.PID.PIVOT.KD.hasChanged(hashCode())) {
      _anglePid.setD(ShooterConstants.PID.PIVOT.KD.get());
    }

    // LOGGING
    Logger.recordOutput("Shooter/Zone", _currentShooterZone.toString());
    Logger.recordOutput("Shooter/AngleDesiredDegrees", _currentShooterZone.getShooterAngle());
    Logger.recordOutput("Shooter/FlywheelSetpoint", this._targetFlywheelSpeed);
    Logger.recordOutput("Mechanism2D/Shooter", _shooterVisualizer.getMechanism());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.shooter.ShooterIO;
import frc.robot.hardware.shooter.ShooterIOInputsAutoLogged;

public class Shooter extends SubsystemBase {
  public enum ShooterZone {
    // Here we define all of the zones for the shooter
    Subwoofer (0, 2.5, 45, 3000, 3000),
    Unknown (-1, -1, 0, 3000, 3000);

    private final double lowBound;
    private final double highBound;
    private final double shooterAngle;
    private final double leftFlywheelSpeed;
    private final double rightFlywheelSpeed;

    ShooterZone(double lowBound, double highBound, double shooterAngle, double leftFlywheelSpeed, double rightFlywheelSpeed) {
        this.lowBound = lowBound;
        this.highBound = highBound;
        this.shooterAngle = shooterAngle;
        this.leftFlywheelSpeed = leftFlywheelSpeed;
        this.rightFlywheelSpeed = rightFlywheelSpeed;
    }

    // These functions can be called on an enum value to get various bits of data
    boolean radiusInZone(double radius) {
      return (radius >= lowBound) || (radius < highBound);
    }

    double getShooterAngle() {
      return this.shooterAngle;
    }

    double getLeftFlywheelSpeed() {
      return this.leftFlywheelSpeed;
    }

    double getRightFlywheelSpeed() {
      return this.rightFlywheelSpeed;
    }
  }

  private final ShooterIO _shooterIO;
  private final ShooterIOInputsAutoLogged _shooterInputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO shooterIO) {
    this._shooterIO = shooterIO;
  }

  public void setTargetPosition(ShooterZone zone) {
    setTargetPositionAsAngle(zone.getShooterAngle());
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
}

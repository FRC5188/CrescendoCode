// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.shooter.ShooterIO;
import frc.robot.hardware.shooter.ShooterIOInputsAutoLogged;

public class Shooter extends SubsystemBase {
  // Note: The channel that this encoder is on will need to be configured for the
  // robot.

  private final ShooterIO _shooterIO;
  private final ShooterIOInputsAutoLogged _shooterInputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO shooterIO) {
    this._shooterIO = shooterIO;
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

  private void runPivotPID() {
    _shooterIO.setTargetPositionAsDegrees(getCurrentPositionInDegrees());
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
}

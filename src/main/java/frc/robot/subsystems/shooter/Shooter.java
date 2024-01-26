// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.shooter.ShooterHardware;

public class Shooter extends SubsystemBase {
  // Note: The channel that this encoder is on will need to be configured for the
  // robot.
  ShooterHardware _hardware;

  public Shooter(ShooterHardware hardware) {
    _hardware = hardware;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getCurrentPositionInDegrees() throws RuntimeException {
    double encoderValueAsRotations = _hardware.getAngleEncoder().get();
    if (encoderValueAsRotations >= ShooterConstants.MAXIMUM_ANGLE_ENCODER_TURNS + Rotation2d.fromDegrees(10).getRotations()
        || encoderValueAsRotations <= ShooterConstants.MINIMUM_ANGLE_ENCODER_TURNS - Rotation2d.fromDegrees(10).getRotations()) {
      throw new RuntimeException(
          "It's impossible for the encoder to be this value. There must be a hardware error. Shut down this subsystem to not break everything.");
    } else {
      return Rotation2d.fromRotations(encoderValueAsRotations).getDegrees();
    }
  }
}

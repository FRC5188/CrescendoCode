// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.shooter.ShooterHardware;

public class Shooter extends SubsystemBase {
  ShooterHardware _hardware;

  public Shooter(ShooterHardware hardware) {
    _hardware = hardware;
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
      _hardware.getAnglePidController().setSetpoint(angle);
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

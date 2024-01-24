// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.shooter.ShooterHardware;

public class Shooter extends SubsystemBase {
  // Note: The channel that this encoder is on will need to be configured for the
  // robot.
  private final DutyCycleEncoder _angleEncoder = configEncoder(new DutyCycleEncoder(0));
  ShooterHardware _hardware;
  private static final float ANGLE_ENCODER_OFFSET_FOR_ROBOT_BASE = 0;

  public Shooter(ShooterHardware hardware) {
    _hardware = hardware;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getCurrentPositionInDegrees() {
    // This assumes that absolute zero is the base of the robot though an offset
    // might need to be set.
    return Rotation2d.fromRotations(_angleEncoder.get()).getDegrees();
  }

  private DutyCycleEncoder configEncoder(DutyCycleEncoder encoder) {
    encoder.setPositionOffset(ANGLE_ENCODER_OFFSET_FOR_ROBOT_BASE);
    return encoder;
  }
}

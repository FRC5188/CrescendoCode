// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.intake.IntakeHardware;

public class Intake extends SubsystemBase {
  private IntakeHardware _hardware;

  public Intake(IntakeHardware hardware) {
    _hardware = hardware;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

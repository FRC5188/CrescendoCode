// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.shooter.ShooterHardware;

public class Shooter extends SubsystemBase {
  ShooterHardware _hardware;

  public Shooter(ShooterHardware hardware) {
    _hardware = hardware;
  }
  private CANSparkFlex configFlywheelMotor (CANSparkFlex flywheelMotor){
      return flywheelMotor;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

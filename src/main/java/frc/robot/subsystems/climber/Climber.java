// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.climber.ClimberHardware;

public class Climber extends SubsystemBase {
  private ClimberHardware _hardware;
  private double _leftEncoderPos;
  private CANSparkFlex  _climberLeft;
  public Climber(ClimberHardware hardware) {
    _hardware = hardware;
  }
private double getLeftEncoderPos(){
  _leftEncoderPos =  _climberLeft.getAbsoluteEncoder(null);
  return _leftEncoderPos
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

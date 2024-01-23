// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.shooter.ShooterHardware;

public class Shooter extends SubsystemBase {
  protected enum ShooterPosition {
        Subwoofer, Podium
    }

  ShooterHardware _hardware;

  public Shooter(ShooterHardware hardware) {
    _hardware = hardware;
  }

public void setFlywheelSpeed(double targetSpeed) {
    // _targetFlywheelSpeed = targetSpeed;
    // _leftFlywheelMotor.set(targetSpeed);
    // _rightFlywheelMotor.set(targetSpeed);
}

public void setTargetPosition(ShooterPosition newPosition) {
    // _targetPosition = newPosition;
    // switch (_targetPosition) {
    //     case Subwoofer:
    //         _anglePIDController.setSetpoint(SUBWOOFER_ANGLE);
    //         break;
    //     case Podium:
    //         _anglePIDController.setSetpoint(PODIUM_ANGLE);
    //         break;
    // }
}
  public void setTargetPositionAsAngle(double angle) {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

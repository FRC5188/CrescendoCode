// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    protected double _targetFlywheelSpeed;
    protected CANSparkFlex _leftFlywheelMotor;
    protected CANSparkFlex _rightFlywheelMotor;

    public void setFlywheelSpeed(double targetSpeed) {
        _targetFlywheelSpeed = targetSpeed;
        _leftFlywheelMotor.set(targetSpeed);
        _rightFlywheelMotor.set(targetSpeed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

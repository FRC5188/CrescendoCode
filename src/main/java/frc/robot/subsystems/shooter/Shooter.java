// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  protected enum ShooterPosition {
        Subwoofer, Podium
    }
    protected final double SUBWOOFER_ANGLE = 0; //subject to change!
    protected final double PODIUM_ANGLE = 0; //also subject to change
    protected final int LEFT_FLYWHEEL_MOTOR_ID = 0;
    protected final int RIGHT_FLYWHEEL_MOTOR_ID = 0;
    protected final double ANGLE_MOTOR_KP = 0;
    protected final double ANGLE_MOTOR_KI = 0;
    protected final double ANGLE_MOTOR_KD = 0;

    protected double _targetFlywheelSpeed;
    protected CANSparkFlex _leftFlywheelMotor;
    protected CANSparkFlex _rightFlywheelMotor;
    protected ShooterPosition _targetPosition;
    protected PIDController _anglePIDController;

    public Shooter() {
        _targetFlywheelSpeed = 0;
        _leftFlywheelMotor = new CANSparkFlex(LEFT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        _rightFlywheelMotor = new CANSparkFlex(RIGHT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        _targetPosition = ShooterPosition.Podium; //change start position
        _anglePIDController = new PIDController(ANGLE_MOTOR_KP, ANGLE_MOTOR_KI, ANGLE_MOTOR_KD);
    }

    public void setFlywheelSpeed(double targetSpeed) {
        _targetFlywheelSpeed = targetSpeed;
        _leftFlywheelMotor.set(targetSpeed);
        _rightFlywheelMotor.set(targetSpeed);
    }

    public void setTargetPosition(ShooterPosition newPosition) {
        _targetPosition = newPosition;
        switch (_targetPosition) {
            case Subwoofer:
                _anglePIDController.setSetpoint(SUBWOOFER_ANGLE);
                break;
            case Podium:
                _anglePIDController.setSetpoint(PODIUM_ANGLE);
                break;
        }
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

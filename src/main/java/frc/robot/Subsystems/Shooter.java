package frc.robot.Subsystems;

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

}

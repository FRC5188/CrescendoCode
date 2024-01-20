package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    protected enum ShooterPosition {
        Subwoofer, Podium
    }

    protected double _targetFlywheelSpeed;
    protected CANSparkFlex _leftFlywheelMotor;
    protected CANSparkFlex _rightFlywheelMotor;
    protected ShooterPosition _targetPosition;
    protected final double SUBWOOFER_ANGLE = 0; //subject to change!
    protected final double PODIUM_ANGLE = 0; //also subject to change
    protected PIDController _anglePIDController;

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

}

package frc.robot.hardware.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.hardware.HardwareConstants;

public class RealShooterHardware implements ShooterHardware {
    private CANSparkFlex _angleMotor;
    private CANSparkFlex _topFlywheelMotor;
    private CANSparkFlex _bottomFlywheelMotor;
    private PIDController _anglePidController;

    public RealShooterHardware() {
        _angleMotor = new CANSparkFlex(HardwareConstants.CanIds.ANGLE_MOTOR_ID, MotorType.kBrushless);
        _topFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.TOP_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        _bottomFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.BOTTOM_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        _anglePidController = new PIDController(0, 0, 0);
    }

    @Override
    public CANSparkFlex getTopFlywheelMotor() {
        return _topFlywheelMotor;
    }

    @Override
    public CANSparkFlex getBottomFlywheelMotor() {
        return _bottomFlywheelMotor;
    }

    @Override
    public CANSparkFlex getAngleMotor() {
        return _angleMotor;
    }

    @Override
    public PIDController getAnglePidController() {
        return _anglePidController;
    }

}

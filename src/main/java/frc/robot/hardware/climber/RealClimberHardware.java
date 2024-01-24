package frc.robot.hardware.climber;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.hardware.HardwareConstants;

public class RealClimberHardware implements ClimberHardware {
    private CANSparkFlex _leftClimberMotor;
    private CANSparkFlex _rightClimberMotor;

    public RealClimberHardware() {
        _leftClimberMotor = new CANSparkFlex(HardwareConstants.CanIds.LEFT_CLIMBER_MOTOR, MotorType.kBrushless);
        _rightClimberMotor = new CANSparkFlex(HardwareConstants.CanIds.RIGHT_CLIMBER_MOTOR, MotorType.kBrushless);
    }

    @Override
    public CANSparkFlex getLeftClimberMotor() {
        return _leftClimberMotor;
    }

    @Override
    public CANSparkFlex getRightClimberMotor() {
        return _rightClimberMotor;
    }

}

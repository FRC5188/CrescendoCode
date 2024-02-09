package frc.robot.hardware.climber;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.hardware.HardwareConstants;

public class RealClimberIO implements ClimberIO {
    private CANSparkFlex _leftClimberMotor;
    private CANSparkFlex _rightClimberMotor;

    public RealClimberIO() {
        _leftClimberMotor = getConfiguredLeftClimberMotor();
        _rightClimberMotor = getConfiguredRightClimberMotor();
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs._leftClimberMotorTemperature = _leftClimberMotor.getMotorTemperature();
        inputs._leftClimberMotorVelocityRotationsPerMin = _leftClimberMotor.getEncoder().getVelocity();
        inputs._leftClimberMotorPositionRotations = _leftClimberMotor.getEncoder().getPosition();
        inputs._leftClimberMotorVoltage = _leftClimberMotor.getAppliedOutput() * _leftClimberMotor.getBusVoltage();
        inputs._leftClimberMotorCurrent = _leftClimberMotor.getOutputCurrent();
        inputs._leftClimberSpeed = _leftClimberMotor.get();

        inputs._rightClimberMotorTemperature = _rightClimberMotor.getMotorTemperature();
        inputs._rightClimberMotorVelocityRotationsPerMin = _rightClimberMotor.getEncoder().getVelocity();
        inputs._rightClimberMotorPositionRotations = _rightClimberMotor.getEncoder().getPosition();
        inputs._rightClimberMotorVoltage = _rightClimberMotor.getAppliedOutput() * _rightClimberMotor.getBusVoltage();
        inputs._rightClimberMotorCurrent = _rightClimberMotor.getOutputCurrent();
        inputs._rightClimberSpeed = _rightClimberMotor.get();
    }

    public void setLeftClimberSpeed(double speed) {
        _leftClimberMotor.set(speed);
    }

    public void setRightClimberSpeed(double speed) {
        _rightClimberMotor.set(speed);
    }

    private CANSparkFlex getConfiguredLeftClimberMotor() {
        // While this method does end up setting the motor equal to itself it is better than having to make a seperate local copy of the motor then returning it. 
        _leftClimberMotor = new CANSparkFlex(HardwareConstants.CanIds.LEFT_CLIMBER_MOTOR, MotorType.kBrushless);
        
        _leftClimberMotor.setCANTimeout(100);
        _leftClimberMotor.setSmartCurrentLimit(40);
        _leftClimberMotor.setSecondaryCurrentLimit(55); // This will use an on/off switch to limit current. Not smart, but if we get above 55 amps we have a problem.

        _leftClimberMotor.enableVoltageCompensation(12); // Even if the battery isn't 12V we'll compensate for it.

        _leftClimberMotor.setInverted(false);

        return _leftClimberMotor;
    }

    private CANSparkFlex getConfiguredRightClimberMotor() {
        _rightClimberMotor = new CANSparkFlex(HardwareConstants.CanIds.LEFT_CLIMBER_MOTOR, MotorType.kBrushless);
        
        _rightClimberMotor.setCANTimeout(100);
        _rightClimberMotor.setSmartCurrentLimit(40);
        _rightClimberMotor.setSecondaryCurrentLimit(55); // This will use an on/off switch to limit current. Not smart, but if we get above 55 amps we have a problem.

        _rightClimberMotor.enableVoltageCompensation(12); // Even if the battery isn't 12V we'll compensate for it.

        _rightClimberMotor.setInverted(false);

        return _rightClimberMotor;
    }
}

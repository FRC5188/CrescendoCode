package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.HardwareConstants;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class RealIntakeIO implements IntakeIO {
    private CANSparkFlex _pivotMotor;
    private CANSparkFlex _rollerMotor;
    private SparkAbsoluteEncoder _pivotMotorEncoder;
    private DigitalInput _leftLimitSwitch;
    private DigitalInput _rightLimitSwitch;

    public RealIntakeIO() {
        configPivotMotor();
        configRollerMotor();
        configEncoder();
        configPivotPID(IntakeConstants.PIVOT_PID_KP,
                        IntakeConstants.PIVOT_PID_KI,
                        IntakeConstants.PIVOT_PID_KD);
        _leftLimitSwitch = new DigitalInput(HardwareConstants.DIOPorts.LEFT_LIMIT_SWITCH_PORT);
        _rightLimitSwitch = new DigitalInput(HardwareConstants.DIOPorts.RIGHT_LIMIT_SWITCH_PORT);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs._pivotMotorTemperature = _pivotMotor.getMotorTemperature();
        inputs._pivotMotorVelocityRotationsPerMin = _pivotMotor.getEncoder().getVelocity();
        inputs._pivotMotorVoltage = _pivotMotor.getAppliedOutput() * _pivotMotor.getBusVoltage();
        inputs._pivotMotorCurrent = _pivotMotor.getOutputCurrent();
        inputs._pivotEncoderPositionDegrees = _pivotMotorEncoder.getPosition();

        inputs._rollerMotorTemperature = _rollerMotor.getMotorTemperature();
        inputs._rollerMotorVelocityRotationsPerMin = _rollerMotor.getEncoder().getVelocity();
        inputs._rollerMotorVoltage = _rollerMotor.getAppliedOutput() * _rollerMotor.getBusVoltage();
        inputs._rollerMotorCurrent = _rollerMotor.getOutputCurrent();

        inputs._leftLimitSwitchIsPushed = _leftLimitSwitch.get();
        inputs._rightLimitSwitchIsPushed = _rightLimitSwitch.get();
    }

    public void setTargetPositionAsDegrees(double degrees) {
        _pivotMotor.getPIDController().setReference(degrees, ControlType.kPosition);
    }
      
    public void setRollerMotorSpeed(double speed) {
        _rollerMotor.set(speed);
    }

    public void setPivotMotorSpeed(double speed) {
        _pivotMotor.set(speed);
    }

    private void configPivotPID(double p, double i, double d) {
        _pivotMotor.getPIDController().setFeedbackDevice(_pivotMotorEncoder);
        _pivotMotor.getPIDController().setP(p);
        _pivotMotor.getPIDController().setI(i);
        _pivotMotor.getPIDController().setD(d);
    }

    private void configPivotMotor() {
        _pivotMotor = new CANSparkFlex(HardwareConstants.CanIds.PIVOT_MOTOR_ID, MotorType.kBrushless);
        
        _pivotMotor.setCANTimeout(100);
        _pivotMotor.setInverted(true);
        _pivotMotor.setIdleMode(IdleMode.kBrake);

        _pivotMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_PIVOT_SMART_CURRENT_LIMIT);
        _pivotMotor.setSecondaryCurrentLimit(IntakeConstants.INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT);

        _pivotMotor.enableVoltageCompensation(12.0);
    }

    private void configRollerMotor() {
        _rollerMotor = new CANSparkFlex(HardwareConstants.CanIds.ROLLER_MOTOR_ID, MotorType.kBrushless);

        _rollerMotor.setCANTimeout(100);
        _rollerMotor.setInverted(true);
        _rollerMotor.setIdleMode(IdleMode.kBrake);

        _rollerMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_ROLLER_SMART_CURRENT_LIMIT);
        _rollerMotor.setSecondaryCurrentLimit(IntakeConstants.INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT);

        _rollerMotor.enableVoltageCompensation(12.0);
    }

    private void configEncoder() {
        _pivotMotorEncoder = _pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        _pivotMotorEncoder.setPositionConversionFactor(360);
        _pivotMotorEncoder.setZeroOffset(HardwareConstants.AbsEncoderOffsets.INTAKE_PIVOT_ENCODER_OFFSET_IN_DEGREES);
    }
}

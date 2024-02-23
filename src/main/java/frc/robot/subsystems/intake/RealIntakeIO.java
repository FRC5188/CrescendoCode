package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.HardwareConstants;

public class RealIntakeIO implements IntakeIO {
    private CANSparkFlex _pivotMotor;
    private CANSparkFlex _rollerMotor;
    private DutyCycleEncoder _pivotMotorEncoder;

    public RealIntakeIO() {
        configPivotMotor();
        configRollerMotor();
        configEncoder();
        configPivotPID(IntakeConstants.PID.ROLLERS.KD,
                        IntakeConstants.PID.ROLLERS.KI,
                        IntakeConstants.PID.ROLLERS.KD);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs._pivotMotorTemperature = _pivotMotor.getMotorTemperature();
        inputs._pivotMotorVelocityRotationsPerMin = _pivotMotor.getEncoder().getVelocity();
        inputs._pivotMotorVoltage = _pivotMotor.getAppliedOutput() * _pivotMotor.getBusVoltage();
        inputs._pivotMotorCurrent = _pivotMotor.getOutputCurrent();
        inputs._pivotEncoderPositionDegrees = (_pivotMotorEncoder.getAbsolutePosition() * 360) - 
                                HardwareConstants.AbsEncoderOffsets.INTAKE_PIVOT_ENCODER_OFFSET_IN_DEGREES;

        inputs._rollerMotorTemperature = _rollerMotor.getMotorTemperature();
        inputs._rollerMotorVelocityRotationsPerMin = _rollerMotor.getEncoder().getVelocity();
        inputs._rollerMotorVoltage = _rollerMotor.getAppliedOutput() * _rollerMotor.getBusVoltage();
        inputs._rollerMotorCurrent = _rollerMotor.getOutputCurrent();
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

    public void configPivotPID(double p, double i, double d) {
        // _pivotMotor.getPIDController().setFeedbackDevice(_pivotMotorEncoder);
        _pivotMotor.getPIDController().setP(p);
        _pivotMotor.getPIDController().setI(i);
        _pivotMotor.getPIDController().setD(d);
    }

    private void configPivotMotor() {
        _pivotMotor = new CANSparkFlex(HardwareConstants.CanIds.PIVOT_MOTOR_ID, MotorType.kBrushless);
        
        _pivotMotor.setCANTimeout(100);
        _pivotMotor.setInverted(true);
        _pivotMotor.setIdleMode(IdleMode.kBrake);

        _pivotMotor.setSmartCurrentLimit(Double.valueOf(IntakeConstants.SOFTWARE.INTAKE_PIVOT_SMART_CURRENT_LIMIT.get()).intValue());
        _pivotMotor.setSecondaryCurrentLimit(IntakeConstants.SOFTWARE.INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT.get());

        _pivotMotor.enableVoltageCompensation(12.0);
    }

    private void configRollerMotor() {
        _rollerMotor = new CANSparkFlex(HardwareConstants.CanIds.ROLLER_MOTOR_ID, MotorType.kBrushless);

        _rollerMotor.setCANTimeout(100);
        _rollerMotor.setInverted(true);
        _rollerMotor.setIdleMode(IdleMode.kBrake);

        _rollerMotor.setSmartCurrentLimit(Double.valueOf(IntakeConstants.SOFTWARE.INTAKE_ROLLER_SMART_CURRENT_LIMIT.get()).intValue());
        _rollerMotor.setSecondaryCurrentLimit(IntakeConstants.SOFTWARE.INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT.get());

        _rollerMotor.enableVoltageCompensation(12.0);
    }

    private void configEncoder() {
        // _pivotMotorEncoder = _pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        // _pivotMotorEncoder.setPositionConversionFactor(360);
        // _pivotMotorEncoder.setZeroOffset(HardwareConstants.AbsEncoderOffsets.INTAKE_PIVOT_ENCODER_OFFSET_IN_DEGREES);
        _pivotMotorEncoder = new DutyCycleEncoder(HardwareConstants.DIOPorts.INTAKE_PIVOT_ENCODER_PORT);
    }
}

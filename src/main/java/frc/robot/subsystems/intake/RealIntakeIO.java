package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
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
    private DigitalInput _distanceSensor;
    private CANSparkMax _feederMotor;

    public RealIntakeIO() {
        configPivotMotor();
        configRollerMotor();
        configFeederMotor();
        configEncoder();
        configPivotPID(IntakeConstants.PIVOT_PID_KP.get(),
                        IntakeConstants.PIVOT_PID_KI.get(),
                        IntakeConstants.PIVOT_PID_KD.get());
        _leftLimitSwitch = new DigitalInput(HardwareConstants.DIOPorts.LEFT_LIMIT_SWITCH_PORT);
        _rightLimitSwitch = new DigitalInput(HardwareConstants.DIOPorts.RIGHT_LIMIT_SWITCH_PORT);
        _distanceSensor = new DigitalInput(HardwareConstants.DIOPorts.INTAKE_DISTANCE_SENSOR_PORT);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        updatedLoggableConstants();

        inputs._pivotMotorTemperature = _pivotMotor.getMotorTemperature();
        inputs._pivotMotorVelocityRotationsPerMin = _pivotMotor.getEncoder().getVelocity();
        inputs._pivotMotorVoltage = _pivotMotor.getAppliedOutput() * _pivotMotor.getBusVoltage();
        inputs._pivotMotorCurrent = _pivotMotor.getOutputCurrent();
        inputs._pivotEncoderPositionDegrees = _pivotMotorEncoder.getPosition();

        inputs._rollerMotorTemperature = _rollerMotor.getMotorTemperature();
        inputs._rollerMotorVelocityRotationsPerMin = _rollerMotor.getEncoder().getVelocity();
        inputs._rollerMotorVoltage = _rollerMotor.getAppliedOutput() * _rollerMotor.getBusVoltage();
        inputs._rollerMotorCurrent = _rollerMotor.getOutputCurrent();

        inputs._leftLimitSwitchIsPushed = !_leftLimitSwitch.get();
        inputs._rightLimitSwitchIsPushed = !_rightLimitSwitch.get();
        // Check if this boolean is true or false when triggered
        inputs._isDistanceSensorTriggered = _distanceSensor.get();

        inputs._feederVoltage = _feederMotor.getAppliedOutput() * _feederMotor.getBusVoltage();
        inputs._feederSpeed = _feederMotor.get();
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

    public void updatedLoggableConstants() {
        if (IntakeConstants.PIVOT_PID_KP.hasChanged(hashCode())) {
            _pivotMotor.getPIDController().setP(IntakeConstants.PIVOT_PID_KP.get());
        }
        if (IntakeConstants.PIVOT_PID_KI.hasChanged(hashCode())) {
            _pivotMotor.getPIDController().setI(IntakeConstants.PIVOT_PID_KI.get());
        }
        if (IntakeConstants.PIVOT_PID_KD.hasChanged(hashCode())) {
            _pivotMotor.getPIDController().setD(IntakeConstants.PIVOT_PID_KD.get());
        }
    }

    public void setFeederMotorSpeed(double speed) {
        _feederMotor.set(speed);
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

    private void configFeederMotor() {
        _feederMotor = new CANSparkMax(HardwareConstants.CanIds.FEEDER_MOTOR_ID, MotorType.kBrushless);

        _feederMotor.enableVoltageCompensation(12.0);
        _feederMotor.setInverted(false);
        _feederMotor.setCANTimeout(100);

        // _feederMotor.setSmartCurrentLimit(40);
        // _feederMotor.setSecondaryCurrentLimit(40);
    }

    private void configEncoder() {
        _pivotMotorEncoder = _pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        _pivotMotorEncoder.setPositionConversionFactor(360);
        _pivotMotorEncoder.setZeroOffset(HardwareConstants.AbsEncoderOffsets.INTAKE_PIVOT_ENCODER_OFFSET_IN_DEGREES);
    }
}

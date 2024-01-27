package frc.robot.hardware.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.hardware.HardwareConstants;

public class IntakeIOSparkFlex implements IntakeIO {
    private CANSparkFlex _pivotMotor;
    private CANSparkMax _rollerMotor;
    private DigitalInput _lightSensor;
    private SparkAbsoluteEncoder _pivotMotorEncoder;

    public IntakeIOSparkFlex() {
        _pivotMotor = new CANSparkFlex(HardwareConstants.CanIds.PIVOT_MOTOR_ID, MotorType.kBrushless);
        _rollerMotor = new CANSparkMax(HardwareConstants.CanIds.ROLLER_MOTOR_ID, MotorType.kBrushless);
        _lightSensor = new DigitalInput(HardwareConstants.DIOPorts.LIGHT_SENSOR_PORT);
        _pivotMotorEncoder = _pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs._pivotMotorTemperature = _pivotMotor.getMotorTemperature();
        inputs._pivotMotorVelocityRotationsPerMin = _pivotMotor.getEncoder().getVelocity();
        inputs._pivotMotorVoltage = _pivotMotor.getAppliedOutput() * _pivotMotor.getBusVoltage();
        inputs._pivotMotorCurrent = _pivotMotor.getOutputCurrent();
        inputs._pivotMotorPositionDegrees = _pivotMotorEncoder.getPosition();

        inputs._rollerMotorTemperature = _rollerMotor.getMotorTemperature();
        inputs._rollerMotorVelocityRotationsPerMin = _rollerMotor.getEncoder().getVelocity();
        inputs._rollerMotorVoltage = _rollerMotor.getAppliedOutput() * _rollerMotor.getBusVoltage();
        inputs._rollerMotorCurrent = _rollerMotor.getOutputCurrent();

        inputs._isLightSensorBlocked = _lightSensor.get();

        inputs._pivotEncoderPositionRotations = _pivotMotorEncoder.getPosition();
    }

    public void configPivotPID(double p, double i, double d) {
        _pivotMotor.getPIDController().setFeedbackDevice(_pivotMotorEncoder);
        _pivotMotor.getPIDController().setP(p);
        _pivotMotor.getPIDController().setI(i);
        _pivotMotor.getPIDController().setD(d);
    }

    public void setTargetPositionAsDegrees(double degrees) {
        _pivotMotor.getPIDController().setReference(Units.degreesToRotations(degrees), ControlType.kPosition);
    }

    public void setRollerMotorSpeed(double speed) {
        _rollerMotor.set(speed);
    }
}

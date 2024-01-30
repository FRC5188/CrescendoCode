package frc.robot.hardware.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import frc.robot.hardware.HardwareConstants;

public class RealShooterIO implements ShooterIO {
    private static final double GEAR_RATIO = 1.0;

    private CANSparkFlex _angleMotor;
    private CANSparkFlex _leftFlywheelMotor;
    private CANSparkFlex _rightFlywheelMotor;
    private SparkAbsoluteEncoder _angleEncoder;

    public RealShooterIO() {
        configAngleMotor();
        configFlywheelMotor();
        _angleEncoder = _angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs._leftFlywheelMotorTemperature = _leftFlywheelMotor.getMotorTemperature();
        inputs._leftFlywheelMotorVelocityRotationsPerMin = (_leftFlywheelMotor.getEncoder().getVelocity()) / GEAR_RATIO;
        inputs._leftFlywheelMotorVoltage = _leftFlywheelMotor.getAppliedOutput() * _leftFlywheelMotor.getBusVoltage();
        inputs._leftFlywheelMotorCurrent = _leftFlywheelMotor.getOutputCurrent();

        inputs._rightFlywheelMotorTemperature = _rightFlywheelMotor.getMotorTemperature();
        inputs._rightFlywheelMotorVelocityRotationsPerMin = (_rightFlywheelMotor.getEncoder().getVelocity()) / GEAR_RATIO;
        inputs._rightFlywheelMotorVoltage = _rightFlywheelMotor.getAppliedOutput() * _rightFlywheelMotor.getBusVoltage();
        inputs._rightFlywheelMotorCurrent = _rightFlywheelMotor.getOutputCurrent();

        inputs._angleMotorTemperature = _angleMotor.getMotorTemperature();
        inputs._angleMotorVelocityRotationsPerMin = _angleMotor.getEncoder().getVelocity();
        inputs._angleMotorVoltage = _angleMotor.getAppliedOutput() * _angleMotor.getBusVoltage();
        inputs._angleMotorCurrent = _angleMotor.getOutputCurrent();
        inputs._angleMotorPositionDegrees = Units.rotationsToDegrees(_angleMotor.getEncoder().getPosition());

        inputs._angleEncoderPositionRotations = _angleEncoder.getPosition(); // TODO: This might be in encoder tics. Decide later what units would be best. 
    }

    public void setLeftFlywheelSpeedRPM(double velocityRotationsPerMinute) {
        _leftFlywheelMotor.getPIDController().setReference(((velocityRotationsPerMinute / 60.0) * GEAR_RATIO), ControlType.kVelocity);
    }

    public void setRightFlywheelSpeedRPM(double velocityRotationsPerMinute) {
        _rightFlywheelMotor.getPIDController().setReference(((velocityRotationsPerMinute / 60.0) * GEAR_RATIO), ControlType.kVelocity);
    }

    public void stopFlywheels() {
        _leftFlywheelMotor.stopMotor();
        _rightFlywheelMotor.stopMotor();
    }

    public void configFlywheelPID(double p, double i, double d) {
        _leftFlywheelMotor.getPIDController().setP(p);
        _leftFlywheelMotor.getPIDController().setI(i);
        _leftFlywheelMotor.getPIDController().setD(d);
        _leftFlywheelMotor.getPIDController().setFF(0, 0);

        _rightFlywheelMotor.getPIDController().setP(p);
        _rightFlywheelMotor.getPIDController().setI(i);
        _rightFlywheelMotor.getPIDController().setD(d);
        _rightFlywheelMotor.getPIDController().setFF(0, 0);
    }

    public void configAnglePID(double p, double i, double d) {
        _angleMotor.getPIDController().setFeedbackDevice(_angleEncoder);
        _angleMotor.getPIDController().setP(p);
        _angleMotor.getPIDController().setI(i);
        _angleMotor.getPIDController().setD(d);
    }

    public void setTargetPositionAsDegrees(double degrees) {
        _angleMotor.getPIDController().setReference(Units.degreesToRotations(degrees), ControlType.kPosition);
    }

    private void configAngleMotor() {
        _angleMotor = new CANSparkFlex(HardwareConstants.CanIds.ANGLE_MOTOR_ID, MotorType.kBrushless);

        _angleMotor.enableVoltageCompensation(12.0);
        _angleMotor.setInverted(true);
        _angleMotor.setCANTimeout(100);

        _angleMotor.setSmartCurrentLimit(40);
        _angleMotor.setSecondaryCurrentLimit(40);
    }

    private void configFlywheelMotor() {
        _leftFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.TOP_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        _rightFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.BOTTOM_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

        _leftFlywheelMotor.enableVoltageCompensation(12.0);
        _leftFlywheelMotor.setInverted(true);
        _leftFlywheelMotor.setCANTimeout(100);

        _rightFlywheelMotor.enableVoltageCompensation(12.0);
        _rightFlywheelMotor.setInverted(false);
        _rightFlywheelMotor.setCANTimeout(100);

        _leftFlywheelMotor.setSmartCurrentLimit(40);
        _leftFlywheelMotor.setSecondaryCurrentLimit(40);

        _rightFlywheelMotor.setSmartCurrentLimit(40);
        _rightFlywheelMotor.setSecondaryCurrentLimit(40);
    }
}
package frc.robot.hardware.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.hardware.HardwareConstants;
import frc.robot.util.MotorFrameConfigurator;

public class RealShooterIO implements ShooterIO {
    private static final double GEAR_RATIO = 1.0;

    private CANSparkFlex _angleMotor;
    private CANSparkFlex _leftFlywheelMotor;
    private CANSparkFlex _rightFlywheelMotor;
    private DutyCycleEncoder _angleEncoder;

    public RealShooterIO() {
        configAngleMotor();
        // configFlywheelMotors();
        configEncoder();
        // configAnglePID(0, 0, 0);
        // configFlywheelPIDs(0, 0, 0);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        // inputs._leftFlywheelMotorTemperature = _leftFlywheelMotor.getMotorTemperature();
        // inputs._leftFlywheelMotorVelocityRotationsPerMin = (_leftFlywheelMotor.getEncoder().getVelocity()) / GEAR_RATIO;
        // inputs._leftFlywheelMotorVoltage = _leftFlywheelMotor.getAppliedOutput() * _leftFlywheelMotor.getBusVoltage();
        // inputs._leftFlywheelMotorCurrent = _leftFlywheelMotor.getOutputCurrent();

        // inputs._rightFlywheelMotorTemperature = _rightFlywheelMotor.getMotorTemperature();
        // inputs._rightFlywheelMotorVelocityRotationsPerMin = (_rightFlywheelMotor.getEncoder().getVelocity())
        //         / GEAR_RATIO;
        // inputs._rightFlywheelMotorVoltage = _rightFlywheelMotor.getAppliedOutput()
        //         * _rightFlywheelMotor.getBusVoltage();
        // inputs._rightFlywheelMotorCurrent = _rightFlywheelMotor.getOutputCurrent();

        inputs._angleMotorTemperature = _angleMotor.getMotorTemperature();
        inputs._angleMotorVelocityRotationsPerMin = _angleMotor.getEncoder().getVelocity();
        inputs._angleMotorVoltage = _angleMotor.getAppliedOutput() * _angleMotor.getBusVoltage();
        inputs._angleMotorCurrent = _angleMotor.getOutputCurrent();
        inputs._angleEncoderPositionDegrees = -((_angleEncoder.getAbsolutePosition() * 360) - HardwareConstants.AbsEncoderOffsets.SHOOTER_ANGLE_ENCODER_OFFSET_IN_DEGREES);
    }

    public void setLeftFlywheelSpeedRPM(double velocityRotationsPerMinute) {
        _leftFlywheelMotor.getPIDController().setReference(((velocityRotationsPerMinute / 60.0) * GEAR_RATIO),
                ControlType.kVelocity);
    }

    public void setRightFlywheelSpeedRPM(double velocityRotationsPerMinute) {
        _rightFlywheelMotor.getPIDController().setReference(((velocityRotationsPerMinute / 60.0) * GEAR_RATIO),
                ControlType.kVelocity);
    }

    public void stopFlywheels() {
        _leftFlywheelMotor.stopMotor();
        _rightFlywheelMotor.stopMotor();
    }

    public void setTargetPositionAsDegrees(double degrees) {
        // TODO: may need an offset to get sensor and input angle to line up
        _angleMotor.getPIDController().setReference(Units.degreesToRotations(degrees), ControlType.kPosition);
    }

    public void setAngleMotorSpeed(double speed) {
        _angleMotor.set(speed);
        System.out.println("This is the speed:" + speed);
    }

    private void configAngleMotor() {
        _angleMotor = new CANSparkFlex(HardwareConstants.CanIds.ANGLE_MOTOR_ID, MotorType.kBrushless);

        _angleMotor.enableVoltageCompensation(12.0);
        _angleMotor.setInverted(true);
        _angleMotor.setCANTimeout(100);

        MotorFrameConfigurator.configDutyCycleSensor(_angleMotor);

        _angleMotor.setSmartCurrentLimit(40);
        _angleMotor.setSecondaryCurrentLimit(40);
    }

    private void configFlywheelMotors() {
        _leftFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.TOP_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        _rightFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.BOTTOM_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

        _leftFlywheelMotor.enableVoltageCompensation(12.0);
        _leftFlywheelMotor.setInverted(true);
        _leftFlywheelMotor.setCANTimeout(100);

        MotorFrameConfigurator.configNoSensor(_leftFlywheelMotor);

        _rightFlywheelMotor.enableVoltageCompensation(12.0);
        _rightFlywheelMotor.setInverted(false);
        _rightFlywheelMotor.setCANTimeout(100);

        MotorFrameConfigurator.configNoSensor(_rightFlywheelMotor);

        _leftFlywheelMotor.setSmartCurrentLimit(40);
        _leftFlywheelMotor.setSecondaryCurrentLimit(40);

        _rightFlywheelMotor.setSmartCurrentLimit(40);
        _rightFlywheelMotor.setSecondaryCurrentLimit(40);
    }

    private void configEncoder() {
        //_angleEncoder = _angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        // _angleEncoder.setPositionConversionFactor(360);
        // _angleEncoder.setZeroOffset(HardwareConstants.AbsEncoderOffsets.SHOOTER_ANGLE_ENCODER_OFFSET_IN_DEGREES);
        _angleEncoder = new DutyCycleEncoder(HardwareConstants.DIOPorts.SHOOTER_ANGLE_ENCODER_PORT);
    }

    private void configFlywheelPIDs(double p, double i, double d) {
        _leftFlywheelMotor.getPIDController().setP(p);
        _leftFlywheelMotor.getPIDController().setI(i);
        _leftFlywheelMotor.getPIDController().setD(d);
        _leftFlywheelMotor.getPIDController().setFF(0, 0);

        _rightFlywheelMotor.getPIDController().setP(p);
        _rightFlywheelMotor.getPIDController().setI(i);
        _rightFlywheelMotor.getPIDController().setD(d);
        _rightFlywheelMotor.getPIDController().setFF(0, 0);
    }

    private void configAnglePID(double p, double i, double d) {
        // _angleMotor.getPIDController().setFeedbackDevice(_angleEncoder);
        // _angleMotor.getPIDController().setP(p);
        // _angleMotor.getPIDController().setI(i);
        // _angleMotor.getPIDController().setD(d);
    }
}

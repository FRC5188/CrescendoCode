package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.HardwareConstants;
import frc.robot.util.MotorFrameConfigurator;

public class RealShooterIO implements ShooterIO {
    private static final double GEAR_RATIO = 1.0;

    private CANSparkFlex _angleMotor;
    private CANSparkFlex _leftFlywheelMotor;
    private CANSparkFlex _rightFlywheelMotor;
    private DutyCycleEncoder _angleEncoder;

    public RealShooterIO() {
        configAngleMotor();
        configFlywheelMotors();
        configEncoder();

        configFlywheelPIDs(0.0001, 0.0000, 0.0000, 0.00022);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        // |================= START LEFT FLYWHEEL MOTOR LOGGING =================|
        inputs._leftFlywheelMotorTemperature = _leftFlywheelMotor.getMotorTemperature();
        inputs._leftFlywheelMotorVelocityRotationsPerMin = (_leftFlywheelMotor.getEncoder().getVelocity()) / GEAR_RATIO;
        inputs._leftFlywheelMotorVoltage = _leftFlywheelMotor.getAppliedOutput() * _leftFlywheelMotor.getBusVoltage();
        inputs._leftFlywheelMotorCurrent = _leftFlywheelMotor.getOutputCurrent();
        // |================= END LEFT FLYWHEEL MOTOR LOGGING =================|

        // |================= START RIGHT FLYWHEEL MOTOR LOGGING =================|

        inputs._rightFlywheelMotorTemperature = _rightFlywheelMotor.getMotorTemperature();
        inputs._rightFlywheelMotorVelocityRotationsPerMin = (_rightFlywheelMotor.getEncoder().getVelocity())
                / GEAR_RATIO;
        inputs._rightFlywheelMotorVoltage = _rightFlywheelMotor.getAppliedOutput()
                * _rightFlywheelMotor.getBusVoltage();
        inputs._rightFlywheelMotorCurrent = _rightFlywheelMotor.getOutputCurrent();
        // |================= END RIGHT FLYWHEEL MOTOR LOGGING =================|

        // |================= START ANGLE MOTOR LOGGING =================|
        inputs._angleMotorTemperature = _angleMotor.getMotorTemperature();
        inputs._angleMotorVelocityRotationsPerMin = _angleMotor.getEncoder().getVelocity();
        inputs._angleMotorVoltage = _angleMotor.getAppliedOutput() * _angleMotor.getBusVoltage();
        inputs._angleMotorCurrent = _angleMotor.getOutputCurrent();
        // |================= END ANGLE MOTOR LOGGING =================|
        
        // |================= START ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|
        inputs._angleEncoderPositionDegrees = -((_angleEncoder.getAbsolutePosition() * 360)
                - HardwareConstants.AbsEncoderOffsets.SHOOTER_ANGLE_ENCODER_OFFSET_IN_DEGREES);
        // |================= END ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|

    }

    /***
     * Set the reference of the PID controller to the RPM provided. The pid controller is set to 
     * velocity control. 
     */
    public void setFlywheelSpeedRPM(double velocityRotationsPerMinute) {
        _leftFlywheelMotor.getPIDController().setReference(((velocityRotationsPerMinute) * GEAR_RATIO),
                ControlType.kVelocity);
    }

    /**
     * Stop the fly wheel motors by calling the stopMotor function.
     */
    public void stopFlywheels() {
        _leftFlywheelMotor.stopMotor();
    }

    /**
     * Set the shooter angle postion
     */
    public void setTargetPositionAsDegrees(double degrees) {
        _angleMotor.getPIDController().setReference(Units.degreesToRotations(degrees), ControlType.kPosition);
    }

    public void setAngleMotorSpeed(double speed) {
        _angleMotor.set(speed);
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
        _leftFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.LEFT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        _rightFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.RIGHT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

        _leftFlywheelMotor.enableVoltageCompensation(12.0);
        _leftFlywheelMotor.setInverted(false);
        _leftFlywheelMotor.setCANTimeout(100);

        MotorFrameConfigurator.configNoSensor(_leftFlywheelMotor);

        _rightFlywheelMotor.enableVoltageCompensation(12.0);
        _rightFlywheelMotor.setCANTimeout(100);

        MotorFrameConfigurator.configNoSensor(_rightFlywheelMotor);

        _leftFlywheelMotor.setSmartCurrentLimit(100);
        _leftFlywheelMotor.setSecondaryCurrentLimit(100);

        _rightFlywheelMotor.setSmartCurrentLimit(100);
        _rightFlywheelMotor.setSecondaryCurrentLimit(100);
        _rightFlywheelMotor.follow(_leftFlywheelMotor, true);
    }

    private void configEncoder() {
        _angleEncoder = new DutyCycleEncoder(HardwareConstants.DIOPorts.SHOOTER_ANGLE_ENCODER_PORT);
    }

    private void configFlywheelPIDs(double p, double i, double d, double f) {
        _leftFlywheelMotor.getPIDController().setP(p);
        _leftFlywheelMotor.getPIDController().setI(i);
        _leftFlywheelMotor.getPIDController().setD(d);
        _leftFlywheelMotor.getPIDController().setFF(f, 0);

        _rightFlywheelMotor.getPIDController().setP(p);
        _rightFlywheelMotor.getPIDController().setI(i);
        _rightFlywheelMotor.getPIDController().setD(d);
        _rightFlywheelMotor.getPIDController().setFF(f, 0);
    }
}

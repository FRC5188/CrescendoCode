package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.HardwareConstants;
import frc.robot.util.MotorFrameConfigurator;

public class RealShooterIO implements ShooterIO {
    private static final double GEAR_RATIO = 1.0;

    private CANSparkFlex _angleMotor;
    private CANSparkFlex _leftFlywheelMotor;
    private CANSparkFlex _rightFlywheelMotor;
    private SparkAbsoluteEncoder _angleEncoder;

    public RealShooterIO() {
        configAngleMotor();
        configFlywheelMotors();
        configEncoder();
        // p: 0.025
        configAnglePID(
            ShooterConstants.SHOOTER_ANGLE_PID_KP.get(),
            ShooterConstants.SHOOTER_ANGLE_PID_KI.get(), 
            ShooterConstants.SHOOTER_ANGLE_PID_KD.get());
        configFlywheelPIDs(
            ShooterConstants.SHOOTER_FLYWHEEL_PID_KP.get(),
            ShooterConstants.SHOOTER_FLYWHEEL_PID_KI.get(), 
            ShooterConstants.SHOOTER_FLYWHEEL_PID_KD.get(), 
            ShooterConstants.SHOOTER_FLYWHEEL_PID_FF.get());
    }

    public void updateInputs(ShooterIOInputs inputs) {
        updatedLoggableConstants();
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
        inputs._angleEncoderPositionDegrees = _angleEncoder.getPosition();
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
        // _angleMotor.getPIDController().setReference(degrees, ControlType.kPosition);
    }

    public void setAngleMotorVoltage(double voltage) {
        _angleMotor.setVoltage(voltage);
    }

    public void setAngleMotorSpeed(double speed){
        _angleMotor.setVoltage(speed);
    }

    private void configAngleMotor() {
        _angleMotor = new CANSparkFlex(HardwareConstants.CanIds.ANGLE_MOTOR_ID, MotorType.kBrushless);

        _angleMotor.enableVoltageCompensation(12.0);
        _angleMotor.setInverted(false);
        _angleMotor.setCANTimeout(100);
        _angleMotor.setIdleMode(IdleMode.kBrake);

        MotorFrameConfigurator.configDutyCycleSensor(_angleMotor);

        _angleMotor.setSmartCurrentLimit(40);
        _angleMotor.setSecondaryCurrentLimit(40);
    }

    private void configFlywheelMotors() {
        _leftFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.LEFT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        _rightFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.RIGHT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

        _leftFlywheelMotor.enableVoltageCompensation(12.0);
        _leftFlywheelMotor.setInverted(true);
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
        _angleEncoder = _angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        _angleEncoder.setPositionConversionFactor(360);
        _angleEncoder.setInverted(true);
        _angleEncoder.setZeroOffset(HardwareConstants.AbsEncoderOffsets.SHOOTER_ANGLE_ENCODER_OFFSET_IN_DEGREES);
    }

    private void configAnglePID(double p, double i, double d) {
        _angleMotor.getPIDController().setFeedbackDevice(_angleEncoder);
        _angleMotor.getPIDController().setP(p);
        _angleMotor.getPIDController().setI(i);
        _angleMotor.getPIDController().setD(d);

        _angleMotor.getPIDController().setIZone(0.4);
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

    private void updatedLoggableConstants(){
        if (ShooterConstants.SHOOTER_FLYWHEEL_PID_KP.hasChanged(hashCode())) {
            _leftFlywheelMotor.getPIDController().setP(ShooterConstants.SHOOTER_FLYWHEEL_PID_KP.get());
            _rightFlywheelMotor.getPIDController().setP(ShooterConstants.SHOOTER_FLYWHEEL_PID_KP.get());
        }
        if (ShooterConstants.SHOOTER_FLYWHEEL_PID_KI.hasChanged(hashCode())) {
            _leftFlywheelMotor.getPIDController().setI(ShooterConstants.SHOOTER_FLYWHEEL_PID_KI.get());
            _rightFlywheelMotor.getPIDController().setI(ShooterConstants.SHOOTER_FLYWHEEL_PID_KI.get());
        }
        if (ShooterConstants.SHOOTER_FLYWHEEL_PID_KD.hasChanged(hashCode())) {
            _leftFlywheelMotor.getPIDController().setD(ShooterConstants.SHOOTER_FLYWHEEL_PID_KD.get());
            _rightFlywheelMotor.getPIDController().setD(ShooterConstants.SHOOTER_FLYWHEEL_PID_KD.get());
        }
        if (ShooterConstants.SHOOTER_FLYWHEEL_PID_FF.hasChanged(hashCode())) {
            _leftFlywheelMotor.getPIDController().setFF(ShooterConstants.SHOOTER_FLYWHEEL_PID_FF.get(), 0);
            _rightFlywheelMotor.getPIDController().setFF(ShooterConstants.SHOOTER_FLYWHEEL_PID_FF.get(), 0);
        }

        if (ShooterConstants.SHOOTER_ANGLE_PID_KP.hasChanged(hashCode())) {
            _angleMotor.getPIDController().setP(ShooterConstants.SHOOTER_ANGLE_PID_KP.get());
        }
        if (ShooterConstants.SHOOTER_ANGLE_PID_KI.hasChanged(hashCode())) {
            _angleMotor.getPIDController().setI(ShooterConstants.SHOOTER_ANGLE_PID_KI.get());
        }
        if (ShooterConstants.SHOOTER_ANGLE_PID_KD.hasChanged(hashCode())) {
            _angleMotor.getPIDController().setD(ShooterConstants.SHOOTER_ANGLE_PID_KD.get());
        }
    }
}

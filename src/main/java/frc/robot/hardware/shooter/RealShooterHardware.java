package frc.robot.hardware.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.hardware.HardwareConstants;
import frc.robot.hardware.HardwareConstants.PIDConstants;

public class RealShooterHardware implements ShooterHardware {
    private CANSparkFlex _angleMotor;
    private CANSparkFlex _topFlywheelMotor;
    private CANSparkFlex _bottomFlywheelMotor;
    private DutyCycleEncoder _angleEncoder;
    private PIDController _anglePIDController;

    public RealShooterHardware() {
        _angleMotor = new CANSparkFlex(HardwareConstants.CanIds.ANGLE_MOTOR_ID, MotorType.kBrushless);
        _topFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.TOP_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        _bottomFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.BOTTOM_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        _angleEncoder = new DutyCycleEncoder(HardwareConstants.DIOPorts.ANGLE_ENCODER_PORT);
        _anglePIDController = new PIDController(PIDConstants.SHOOTER_ANGLE_KP, PIDConstants.SHOOTER_ANGLE_KI, PIDConstants.SHOOTER_ANGLE_KD);
    }

    @Override
    public CANSparkFlex getTopFlywheelMotor() {
        return _topFlywheelMotor;
    }

    @Override
    public CANSparkFlex getBottomFlywheelMotor() {
        return _bottomFlywheelMotor;
    }

    @Override
    public CANSparkFlex getAngleMotor() {
        return _angleMotor;
    }

    @Override
    public DutyCycleEncoder getAngleEncoder() {
        return _angleEncoder;
    }

    @Override
    public PIDController getAnglePIDController() {
        return _anglePIDController;
    }

}

package frc.robot.hardware.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.hardware.HardwareConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class RealShooterHardware implements ShooterHardware {
    private CANSparkFlex _angleMotor;
    private CANSparkFlex _topFlywheelMotor;
    private CANSparkFlex _bottomFlywheelMotor;
    private PIDController _anglePIDController;
    private DutyCycleEncoder _angleEncoder;

    public RealShooterHardware() {
        _angleMotor = new CANSparkFlex(HardwareConstants.CanIds.ANGLE_MOTOR_ID, MotorType.kBrushless);
        _topFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.TOP_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        _bottomFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.BOTTOM_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        _angleEncoder = configureEncoder();
        _anglePIDController = new PIDController(ShooterConstants.ANGLE_PID_P, ShooterConstants.ANGLE_PID_I, ShooterConstants.ANGLE_PID_D);
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
  
      
    public PIDController getAnglePIDController() {
        return _anglePIDController;
    }

    private DutyCycleEncoder configureEncoder() {
        DutyCycleEncoder angleEncoder = new DutyCycleEncoder(HardwareConstants.DIOPorts.SHOOTER_ANGLE_ENCODER_PORT);
        angleEncoder.setPositionOffset(ShooterConstants.ANGLE_ENCODER_OFFSET);
        return angleEncoder;
    }

    private CANSparkFlex configureAngleMotor(int canId) {
        return new CANSparkFlex(canId, MotorType.kBrushless);
    }

    private CANSparkFlex configureFlywheel(int canId) {
        return new CANSparkFlex(canId, MotorType.kBrushless);
    }
  

}

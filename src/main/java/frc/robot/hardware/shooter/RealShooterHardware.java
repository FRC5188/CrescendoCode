package frc.robot.hardware.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.hardware.HardwareConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class RealShooterHardware implements ShooterHardware {
    private final CANSparkFlex _angleMotor;
    private final CANSparkFlex _topFlywheelMotor;
    private final CANSparkFlex _bottomFlywheelMotor;

    private final DutyCycleEncoder _angleEncoder;

    public RealShooterHardware() {
        _angleMotor = configureAngleMotor(HardwareConstants.CanIds.ANGLE_MOTOR_ID);
        _topFlywheelMotor = configureFlywheel(HardwareConstants.CanIds.TOP_FLYWHEEL_MOTOR_ID);
        _bottomFlywheelMotor = configureFlywheel(HardwareConstants.CanIds.BOTTOM_FLYWHEEL_MOTOR_ID);

        _angleEncoder = configureEncoder();

        // Now that we've created the objects some might require additional configuration which will be carried out by helper-methods in this file. 
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

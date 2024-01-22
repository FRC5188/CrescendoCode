package frc.robot.hardware.shooter;

import org.easymock.EasyMock;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.hardware.HardwareConstants;

public class RealShooterHardware implements ShooterHardware {
    private CANSparkFlex _angleMotor;
    private CANSparkFlex _topFlywheelMotor;
    private CANSparkFlex _bottomFlywheelMotor;

    public RealShooterHardware() {
        _angleMotor = new CANSparkFlex(HardwareConstants.CanIds.ANGLE_MOTOR_ID, MotorType.kBrushless);
        _topFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.TOP_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        _bottomFlywheelMotor = new CANSparkFlex(HardwareConstants.CanIds.BOTTOM_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
    }

    public void replayHardware() {
        EasyMock.replay(_angleMotor, _topFlywheelMotor, _bottomFlywheelMotor);
    }

    public void verifyHardware() {
        EasyMock.verify(_angleMotor, _topFlywheelMotor, _bottomFlywheelMotor);
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

}

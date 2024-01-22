package frc.robot.hardware.shooter;

import org.easymock.EasyMock;

import com.revrobotics.CANSparkFlex;

public class SimShooterHardware implements ShooterHardware {
    private CANSparkFlex _angleMotor;
    private CANSparkFlex _topFlywheelMotor;
    private CANSparkFlex _bottomFlywheelMotor;

    public SimShooterHardware() {
        _angleMotor = EasyMock.mock(CANSparkFlex.class);
        _topFlywheelMotor = EasyMock.mock(CANSparkFlex.class);
        _bottomFlywheelMotor = EasyMock.mock(CANSparkFlex.class);
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

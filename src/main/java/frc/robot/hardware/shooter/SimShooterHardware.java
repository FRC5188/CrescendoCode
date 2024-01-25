package frc.robot.hardware.shooter;

import org.easymock.EasyMock;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SimShooterHardware implements ShooterHardware {
    private CANSparkFlex _angleMotor;
    private CANSparkFlex _topFlywheelMotor;
    private CANSparkFlex _bottomFlywheelMotor;
    private DutyCycleEncoder _angleEncoder;

    public SimShooterHardware() {
        _angleMotor = EasyMock.mock(CANSparkFlex.class);
        _topFlywheelMotor = EasyMock.mock(CANSparkFlex.class);
        _bottomFlywheelMotor = EasyMock.mock(CANSparkFlex.class);
        _angleEncoder = EasyMock.mock(DutyCycleEncoder.class);
    }

    public void replayHardware() {
        EasyMock.replay(_angleMotor, _topFlywheelMotor, _bottomFlywheelMotor, _angleEncoder);
    }

    public void verifyHardware() {
        EasyMock.verify(_angleMotor, _topFlywheelMotor, _bottomFlywheelMotor, _angleEncoder);
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
}

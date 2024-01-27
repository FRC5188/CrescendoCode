package frc.robot.hardware.climber;

import org.easymock.EasyMock;

import com.revrobotics.CANSparkFlex;

public class SimClimberHardware implements ClimberIO {
    private CANSparkFlex _leftClimberMotor;
    private CANSparkFlex _rightClimberMotor;

    public SimClimberHardware() {
        _leftClimberMotor = EasyMock.mock(CANSparkFlex.class);
        _rightClimberMotor = EasyMock.mock(CANSparkFlex.class);
    }

    public void replayHardware() {
        EasyMock.replay(_leftClimberMotor, _rightClimberMotor);
    }

    public void verifyHardware() {
        EasyMock.verify(_leftClimberMotor, _rightClimberMotor);
    }

    @Override
    public CANSparkFlex getLeftClimberMotor() {
        return _leftClimberMotor;
    }

    @Override
    public CANSparkFlex getRightClimberMotor() {
        return _rightClimberMotor;
    }
}

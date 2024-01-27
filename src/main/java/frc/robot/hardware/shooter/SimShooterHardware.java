package frc.robot.hardware.shooter;

import org.easymock.EasyMock;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;

public class SimShooterHardware implements ShooterIO {
    private CANSparkFlex _angleMotor;
    private CANSparkFlex _topFlywheelMotor;
    private CANSparkFlex _bottomFlywheelMotor;
    private DutyCycleEncoder _angleEncoder;
    private PIDController _anglePIDController;

    public SimShooterHardware() {
        _angleMotor = EasyMock.mock(CANSparkFlex.class);
        _topFlywheelMotor = EasyMock.mock(CANSparkFlex.class);
        _bottomFlywheelMotor = EasyMock.mock(CANSparkFlex.class);
        _angleEncoder = EasyMock.mock(DutyCycleEncoder.class);
        _anglePIDController = EasyMock.mock(PIDController.class);
    }

    public void replayHardware() {
        EasyMock.replay(_angleMotor, _topFlywheelMotor, _bottomFlywheelMotor, _angleEncoder, _anglePIDController);
    }

    public void verifyHardware() {
        EasyMock.verify(_angleMotor, _topFlywheelMotor, _bottomFlywheelMotor, _angleEncoder, _anglePIDController);
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

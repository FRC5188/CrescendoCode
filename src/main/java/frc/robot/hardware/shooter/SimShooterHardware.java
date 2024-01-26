package frc.robot.hardware.shooter;

import org.easymock.EasyMock;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.controller.PIDController;

public class SimShooterHardware implements ShooterHardware {
    private CANSparkFlex _angleMotor;
    private CANSparkFlex _topFlywheelMotor;
    private CANSparkFlex _bottomFlywheelMotor;
    private PIDController _anglePIDController;

    public SimShooterHardware() {
        _angleMotor = EasyMock.mock(CANSparkFlex.class);
        _topFlywheelMotor = EasyMock.mock(CANSparkFlex.class);
        _bottomFlywheelMotor = EasyMock.mock(CANSparkFlex.class);
        _anglePIDController = EasyMock.mock(PIDController.class);
    }

    public void replayHardware() {
        EasyMock.replay(_angleMotor, _topFlywheelMotor, _bottomFlywheelMotor, _anglePIDController);
    }

    public void verifyHardware() {
        EasyMock.verify(_angleMotor, _topFlywheelMotor, _bottomFlywheelMotor, _anglePIDController);
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
    public PIDController getAnglePIDController() {
        return _anglePIDController;
    }
}

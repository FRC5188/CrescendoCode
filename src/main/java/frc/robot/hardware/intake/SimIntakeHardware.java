package frc.robot.hardware.intake;

import org.easymock.EasyMock;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class SimIntakeHardware implements IntakeHardware {
    private CANSparkFlex _pivotMotor;
    private CANSparkMax _rollerMotor;
    private DigitalInput _lightSensor;

    public SimIntakeHardware() {
        _pivotMotor = EasyMock.mock(CANSparkFlex.class);
        _rollerMotor = EasyMock.mock(CANSparkMax.class);
        _lightSensor = EasyMock.mock(DigitalInput.class);
    }

    public void replayHardware() {
        EasyMock.replay(_pivotMotor, _rollerMotor, _lightSensor);
    }

    public void verifyHardware() {
        EasyMock.verify(_pivotMotor, _rollerMotor, _lightSensor);
    }

    @Override
    public CANSparkFlex getPivotMotor() {
        return _pivotMotor;
    }

    @Override
    public CANSparkMax getRollerMotor() {
        return _rollerMotor;
    }

    @Override
    public DigitalInput getLightSensor() {
        return _lightSensor;
    }
}

package frc.robot.hardware.intake;

import org.easymock.EasyMock;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class SimIntakeHardware implements IntakeIO {
    private CANSparkFlex _pivotMotor;
    private CANSparkMax _rollerMotor;
    private DigitalInput _lightSensor;
    private PIDController _pivotMotorPID;

    public SimIntakeHardware() {
        _pivotMotor = EasyMock.mock(CANSparkFlex.class);
        _rollerMotor = EasyMock.mock(CANSparkMax.class);
        _lightSensor = EasyMock.mock(DigitalInput.class);
        _pivotMotorPID = EasyMock.mock(PIDController.class);
    }

    public void replayHardware() {
        EasyMock.replay(_pivotMotor, _rollerMotor, _lightSensor, _pivotMotorPID);
    }

    public void verifyHardware() {
        EasyMock.verify(_pivotMotor, _rollerMotor, _lightSensor, _pivotMotorPID);
    }
}

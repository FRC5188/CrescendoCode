package frc.robot.hardware.intake;

import org.easymock.EasyMock;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.hardware.HardwareConstants;

public class RealIntakeHardware implements IntakeHardware {
    private CANSparkFlex _pivotMotor;
    private CANSparkMax _rollerMotor;
    private DigitalInput _lightSensor;
    private PIDController _pivotMotorPID;

    public RealIntakeHardware() {
        _pivotMotor = new CANSparkFlex(HardwareConstants.CanIds.PIVOT_MOTOR_ID, MotorType.kBrushless);
        _rollerMotor = new CANSparkMax(HardwareConstants.CanIds.ROLLER_MOTOR_ID, MotorType.kBrushless);
        _lightSensor = new DigitalInput(HardwareConstants.DIOPorts.LIGHT_SENSOR_PORT);
        _pivotMotorPID = new PIDController(0, 0, 0);
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

    @Override
    public PIDController getPivotMotorPID() {
        return _pivotMotorPID;
    }
}

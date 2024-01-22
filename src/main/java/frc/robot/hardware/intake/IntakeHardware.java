package frc.robot.hardware.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public interface IntakeHardware {
    CANSparkFlex getPivotMotor();

    CANSparkMax getRollerMotor();

    DigitalInput getLightSensor();
}

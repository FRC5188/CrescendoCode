package frc.robot.hardware.climber;

import com.revrobotics.CANSparkFlex;

public interface ClimberHardware {
    CANSparkFlex getLeftClimberMotor();

    CANSparkFlex getRightClimberMotor();

}

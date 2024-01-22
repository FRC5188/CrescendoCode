package frc.robot.hardware.shooter;

import com.revrobotics.CANSparkFlex;

public interface ShooterHardware {
    CANSparkFlex getAngleMotor();

    CANSparkFlex getTopFlywheelMotor();

    CANSparkFlex getBottomFlywheelMotor();
}

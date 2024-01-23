package frc.robot.hardware.shooter;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.controller.PIDController;

public interface ShooterHardware {
    CANSparkFlex getAngleMotor();

    CANSparkFlex getTopFlywheelMotor();

    CANSparkFlex getBottomFlywheelMotor();

    PIDController getAnglePidController();
}

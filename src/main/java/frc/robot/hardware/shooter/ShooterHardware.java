package frc.robot.hardware.shooter;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public interface ShooterHardware {
    CANSparkFlex getAngleMotor();

    CANSparkFlex getTopFlywheelMotor();

    CANSparkFlex getBottomFlywheelMotor();

    DutyCycleEncoder getAngleEncoder();

    PIDController getAnglePIDController();
}

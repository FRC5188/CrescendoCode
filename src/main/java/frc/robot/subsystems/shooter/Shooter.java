// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.shooter.ShooterHardware;

public class Shooter extends SubsystemBase {
    public enum ShooterPosition {
        Subwoofer, Podium
    }

    ShooterHardware _hardware;
    ShooterPosition _shooterPosition;

    public Shooter(ShooterHardware hardware) {
        _hardware = hardware;
        _shooterPosition = ShooterPosition.Subwoofer;
    }

    public void setTargetPosition(ShooterPosition position) {
        _shooterPosition = position;
        switch (_shooterPosition) {
            case Subwoofer:
                setTargetPositionAsAngle(ShooterConstants.SUBWOOFER_ANGLE);
                break;
            case Podium:
                setTargetPositionAsAngle(ShooterConstants.PODIUM_ANGLE);
                break;
        }
    }

    public void setTargetPositionAsAngle(Double angle) {
        // Will be implemented in a different branch
        // this method doesn't need anything in it for the tests on this branch to pass
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.climber.ClimberHardware;

public class Climber extends SubsystemBase {
    private ClimberHardware _hardware;
    private boolean _canMove;

    public Climber(ClimberHardware hardware) {
        _hardware = hardware;
    }

    public void setCanMove(boolean canMove) {
        _canMove = canMove;
    }

    public boolean getCanMove(){
        return _canMove;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

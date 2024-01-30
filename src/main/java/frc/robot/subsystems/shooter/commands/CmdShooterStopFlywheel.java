// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import frc.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class CmdShooterStopFlywheel extends Command {
    private Shooter _shooterSubsystem;
    private double _speed;

    public CmdShooterStopFlywheel(Shooter shooterSubsystem, double speed) {
        _shooterSubsystem = shooterSubsystem;
        _speed = speed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //stop motor method
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

public class CmdShooterRunShooter extends Command {

    private Shooter _shooterSubsystem;
    private Intake _intakeSubsystem;
    private ShooterZone _zone;
    private Drive _driveSubsystem;

    public CmdShooterRunShooter(Shooter shooterSubsystem, Intake intakeSubsystem, Drive driveSubsystem) {
        _shooterSubsystem = shooterSubsystem;
        _intakeSubsystem = intakeSubsystem;
        addRequirements(_shooterSubsystem, _intakeSubsystem, _driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (_intakeSubsystem.hasNote()) {
            _zone = _shooterSubsystem.getZoneFromRadius(_driveSubsystem.getRadiusToSpeakerInMeters());
            _shooterSubsystem.runShooterForZone(_zone);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class CmdAutoUpdateZones extends Command {
    private Drive _driveSubsystem;
    private Shooter _shooterSubsystem;

    public CmdAutoUpdateZones(Drive driveSubsystem, Shooter shooterSubsystem) {
        addRequirements(driveSubsystem, shooterSubsystem);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        _shooterSubsystem.setCurrentZone(_shooterSubsystem.getZoneFromRadius(_driveSubsystem.getRadiusToSpeakerInMeters()));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class CmdDriveGoToNote extends Command {
  private final Drive _drive;
  private ChassisSpeeds _chassisSpeeds;

  public CmdDriveGoToNote(Drive drivetrainSubsystem) {

    this._drive = drivetrainSubsystem;

    _chassisSpeeds = _drive._visionDriveIO.getChassisSpeeds();

    // _forwardSpeed = chassisSpeeds.vxMetersPerSecond;
    // _rotSpeed = chassisSpeeds.vyMetersPerSecond;
    // _range = chassisSpeeds.omegaRadiansPerSecond;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    System.out.println("Button A was pressed.");
    _chassisSpeeds = _drive._visionDriveIO.getChassisSpeeds();

    this._drive.runVelocity(_chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    // TODO: Edit
    return false;
  }
}

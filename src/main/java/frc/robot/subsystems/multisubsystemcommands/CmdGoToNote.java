// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.visiondrive.VisionDrive;

public class CmdGoToNote extends Command {
  private final Drive _drive;
  private final VisionDrive _visionDrive;
  private ChassisSpeeds _chassisSpeeds;

  public CmdGoToNote(Drive drivetrainSubsystem, VisionDrive visionDriveSubsystem) {

    this._drive = drivetrainSubsystem;
    this._visionDrive = visionDriveSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Update values.
    _chassisSpeeds = _visionDrive.getChassisSpeedsForDriveToNote();
    this._drive.runVelocity(_chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

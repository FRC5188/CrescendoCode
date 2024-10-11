// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.visiondrive.VisionDrive;

public class CmdGoToNote extends Command {
  private final Drive _drive;
  private final VisionDrive _visionDrive;
  private final Intake _intake;
  private ChassisSpeeds _chassisSpeeds;
  private boolean _isOverLine;

  public CmdGoToNote(Drive drivetrainSubsystem, VisionDrive visionDriveSubsystem, Intake intakeSubsystem) {

    this._drive = drivetrainSubsystem;
    this._visionDrive = visionDriveSubsystem;
    this._intake = intakeSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Stop us from driving in auto if we are over the line
    if (DriverStation.getAlliance().get() != null && DriverStation.isAutonomousEnabled()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        if (_drive.getPose().getX() > DriveConstants.BLUE_AUTO_PENALTY_LINE) {
          System.out.println("Find Note1");
          _isOverLine = true;
        }
      } else {
        if (_drive.getPose().getX() < DriveConstants.RED_AUTO_PENALTY_LINE) {
          System.out.println("Find Note2");
          _isOverLine = true;
        }
      }
    }

    // Update values.
    _chassisSpeeds = _visionDrive.getChassisSpeedsForDriveToNote();
    this._drive.runVelocity(_chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return (DriverStation.isAutonomous() && _intake.hasNote()) || _isOverLine;
  }
}

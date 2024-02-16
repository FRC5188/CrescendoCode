// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class CmdShooterRunPids extends Command {
  /** Creates a new CmdShooterRunPids. */
  private Shooter _shooterSubsystem;

  /**
   * Runs the PIDs for the shooter
   * @param shooterSubsystem
   */
  public CmdShooterRunPids(Shooter shooterSubsystem) {
    _shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  /**
   * When the command is called, prints "CmdShooterRunPIDs"
   */
  public void initialize() {
    System.out.println("CmdShooterRunPIDs");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  /**
   * While the command is in use, print "Running Cmd" and runs angle PID
   */
  public void execute() {
    System.out.println("Running Cmd");    
    _shooterSubsystem.runAnglePid();
  }

  // Called once the command ends or is interrupted.
  @Override
  /**
   * Ends the command when desired or interrupted
   */
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  /**
   * Returns true if the command is done
   */
  public boolean isFinished() {
    return false;
  }
}

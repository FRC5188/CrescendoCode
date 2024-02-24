// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

/**
 * <STRONG>Purpose:</STRONG> Should be ran once in the initialize method for shooting. Once this is ran it'll run PID continuously since the command 
 * is never finished. </p>
 * <STRONG>Status:</STRONG> Working
 */
public class CmdShooterRunPids extends Command {
  private Shooter _shooterSubsystem;

  public CmdShooterRunPids(Shooter shooterSubsystem) {
    this._shooterSubsystem = shooterSubsystem;

    addRequirements(this._shooterSubsystem);
  }

  /**<STRONG>Purpose:</STRONG> Will continuously run the PID which should calculate and update the motor speed. */
  @Override
  public void execute() {
    _shooterSubsystem.runAnglePid();
  }

  @Override
  public boolean isFinished() {
    return false; // Never stops running.
  }
}

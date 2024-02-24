// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

/**<STRONG>Purpose:</STRONG> Will update the target setpoint for the zone that our robot is in based on it's pose. */
/**<STRONG>Status:</STRONG> Working */
public class CmdShooterRunShooterForZone extends Command {
  private Shooter _shooterSubsystem;
  private ShooterZone _zone;

  public CmdShooterRunShooterForZone(Shooter shooterSubsystem, ShooterZone zone) {
    this._shooterSubsystem = shooterSubsystem;
    this._zone = zone;

    addRequirements(this._shooterSubsystem);
  }

  /**<STRONG>Purpose:</STRONG> Updates the setpoint of the PID meaning it should run only once.*/
  @Override
  public void initialize() {
    this._shooterSubsystem.runShooterForZone(_zone);
  }

  @Override
  public boolean isFinished() {
    return true; // Is only run once.
  }
}

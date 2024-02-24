// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

/**<STRONG>Purpose:</STRONG> When call it'll enable the autoshoot function.*/
/**<STRONG>Status:</STRONG> Working */
public class CmdShooterSetAutoshootEnabled extends Command {
  private boolean _enabled;
  private Shooter _shooterSubsystem;

  /**
   * @param enabled True Enables Autoshooting, False Disables Autoshooting
   */
  public CmdShooterSetAutoshootEnabled(Shooter shooterSubsystem, boolean enabled) {
    this._shooterSubsystem = shooterSubsystem;
    this._enabled = enabled;

    addRequirements(shooterSubsystem);
  }

  /**<STRONG>Purpose:</STRONG> Enables/Disables the autoshooting functionality based on the boolean. Should only be run since we're just updating a value.*/
  @Override
  public void initialize() {
    _shooterSubsystem.setAutoShootEnabled(_enabled);
  }

  @Override
  public boolean isFinished() {
    return true; // This should only be ran once.
  }
}

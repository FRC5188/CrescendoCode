// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

/**
 * Set the flywheel speed based on the zone passed into the command.
 * This command is meant to be used with a button press. It will not stop by default.
 * Use this command with the button.whileTrue function.
 * 
 */
public class CmdShooterRunFlywheelsForZone extends Command {
  /** Creates a new CmdShooterRunShooterForZone. */
  private Shooter _shooterSubsystem;
  private ShooterZone _zone;

  /**
   * Set the shooter angle and flywheel speed for a zone.
   * 
   * @param shooterSubsystem
   * @param zone
   */
  public CmdShooterRunFlywheelsForZone(Shooter shooterSubsystem, ShooterZone zone) {
    // Use addRequirements() here to declare subsystem dependencies.
    _shooterSubsystem = shooterSubsystem;
    _zone = zone;
    addRequirements(_shooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _shooterSubsystem.setFlywheelSpeed(_zone.getLeftFlywheelSpeed());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooterSubsystem.stopFlywheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

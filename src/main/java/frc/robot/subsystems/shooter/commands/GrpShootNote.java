// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.commands.CmdIntakeRollersSpit;
import frc.robot.subsystems.intake.commands.GrpIntakeMoveToPosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrpShootNote extends SequentialCommandGroup {
  /** Creates a new GrpShootNote. */
  public GrpShootNote(Shooter shooterSubsystem, Intake intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(shooterSubsystem, intakeSubsystem);
    addCommands(
    new CmdIntakeRollersSpit(intakeSubsystem),  
    new CmdShooterRunShooterForZone (shooterSubsystem, ShooterZone.Unknown),
    new GrpIntakeMoveToPosition (intakeSubsystem, IntakePosition.Stowed));
  }



}


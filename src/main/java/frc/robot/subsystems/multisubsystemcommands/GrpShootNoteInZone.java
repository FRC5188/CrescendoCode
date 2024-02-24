package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.commands.CmdIntakeRollersSpit;
import frc.robot.subsystems.intake.commands.GrpIntakeMoveToPosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

public class GrpShootNoteInZone extends SequentialCommandGroup {

    public GrpShootNoteInZone(Intake intakeSubsystem, Shooter shooterSubsystem, ShooterZone zone) {

        addRequirements(intakeSubsystem, shooterSubsystem);

        addCommands(
                shooterSubsystem.buildCommand().setAutoShooting(false),
                shooterSubsystem.buildCommand().runForZone(zone),
                shooterSubsystem.buildCommand().waitUntilReady(intakeSubsystem),
                new CmdIntakeRollersSpit(intakeSubsystem),
                shooterSubsystem.buildCommand().runForZone(zone),
                new GrpIntakeMoveToPosition(intakeSubsystem, IntakePosition.Stowed));

    }

}

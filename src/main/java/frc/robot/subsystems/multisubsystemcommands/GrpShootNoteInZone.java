package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.commands.CmdIntakeRollersSpit;
import frc.robot.subsystems.intake.commands.GrpIntakeMoveToPosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;
import frc.robot.subsystems.shooter.commands.CmdShooterRunShooterForZone;
import frc.robot.subsystems.shooter.commands.CmdShooterWaitUntilReady;

public class GrpShootNoteInZone extends SequentialCommandGroup {

    /**
     * prepares shooter to spit a note
     * @param intakeSubsystem
     * @param shooterSubsystem
     * @param zone
     */
    public GrpShootNoteInZone(Intake intakeSubsystem, Shooter shooterSubsystem, ShooterZone zone) {

        addRequirements(intakeSubsystem, shooterSubsystem);

        addCommands(
                new CmdShooterRunShooterForZone(shooterSubsystem, zone),
                new CmdShooterWaitUntilReady(shooterSubsystem, intakeSubsystem),
                new CmdIntakeRollersSpit(intakeSubsystem),
                new CmdShooterRunShooterForZone(shooterSubsystem, ShooterZone.Unknown),
                new GrpIntakeMoveToPosition(intakeSubsystem, IntakePosition.Stowed));

    }

}

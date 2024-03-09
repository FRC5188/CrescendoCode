package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;
import frc.robot.subsystems.shooter.commands.CmdShooterRunShooterForZone;
import frc.robot.subsystems.shooter.commands.CmdShooterSetAutoshootEnabled;
import frc.robot.subsystems.shooter.commands.CmdShooterWaitUntilReady;

public class GrpShootNoteInZone extends SequentialCommandGroup {

    public GrpShootNoteInZone(Intake intakeSubsystem, Shooter shooterSubsystem, ShooterZone zone) {

        addRequirements(intakeSubsystem, shooterSubsystem);

        System.out.print("Zone: ");
        System.out.println(zone);

        addCommands(
                new CmdShooterSetAutoshootEnabled(shooterSubsystem, false),
                intakeSubsystem.buildCommand().setPosition(IntakePosition.Stowed),
                new CmdShooterRunShooterForZone(shooterSubsystem, zone),
                new CmdShooterWaitUntilReady(shooterSubsystem, intakeSubsystem),
                intakeSubsystem.buildCommand().spit(IntakeConstants.ROLLERS.SPIT_TIME.get()),                
                new CmdShooterRunShooterForZone(shooterSubsystem, ShooterZone.Unknown));
                intakeSubsystem.buildCommand().setPosition(IntakePosition.Stowed);
    }

}

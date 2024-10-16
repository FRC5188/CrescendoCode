package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;
import frc.robot.subsystems.shooter.commands.CmdShooterWaitUntilReady;

public class GrpShootNoteInZone extends SequentialCommandGroup {
    
    private boolean _wasAutoShootEnabled;
    public GrpShootNoteInZone(Intake intakeSubsystem, Shooter shooterSubsystem, ShooterZone zone) {

        addRequirements(intakeSubsystem, shooterSubsystem);
        _wasAutoShootEnabled = shooterSubsystem.isAutoShootEnabled();

        addCommands(
                shooterSubsystem.buildCommand().setAutoShootEnabled(false),
                intakeSubsystem.buildCommand().setPosition(IntakePosition.Stowed),
                shooterSubsystem.buildCommand().runForZone(zone),
                new CmdShooterWaitUntilReady(shooterSubsystem).withTimeout(2),
                intakeSubsystem.buildCommand().spit(IntakeConstants.INTAKE_SPIT_TIME.get()),
                shooterSubsystem.buildCommand().runForZone(ShooterZone.Unknown),
        new InstantCommand(() -> shooterSubsystem.setAutoShootEnabled(_wasAutoShootEnabled)));
    }

}

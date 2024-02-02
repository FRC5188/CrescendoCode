package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;

public class CmdShooterWaitUntilReady extends Command {
    private Shooter _shooterSubsystem;
    private Intake _intakeSubsystem;

    public CmdShooterWaitUntilReady(Shooter shooterSubsystem, Intake intakeSubsystem) {
        _shooterSubsystem = shooterSubsystem;
        _intakeSubsystem = intakeSubsystem;

        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public boolean isFinished() {
        return _shooterSubsystem.isReady() && 
        _intakeSubsystem.pivotAtSetpoint() && 
        (_intakeSubsystem.getIntakePosition() == IntakePosition.AmpScore || _intakeSubsystem.getIntakePosition() == IntakePosition.SpeakerScore);
    }
}

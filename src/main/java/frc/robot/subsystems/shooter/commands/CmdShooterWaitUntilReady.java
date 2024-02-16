package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;

public class CmdShooterWaitUntilReady extends Command {
    private Shooter _shooterSubsystem;
    private Intake _intakeSubsystem;

    /**
     * It sees if the shooter and intake subsystems are ready
     * @param shooterSubsystem
     * @param intakeSubsystem
     */
    public CmdShooterWaitUntilReady(Shooter shooterSubsystem, Intake intakeSubsystem) {
        _shooterSubsystem = shooterSubsystem;
        _intakeSubsystem = intakeSubsystem;

        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    /**
     * It sees if the shooter is ready and if the intake is at the right position and pivot point
     * If they both are true the it stop the series of code
     */
    public boolean isFinished() {
        return _shooterSubsystem.isReady() && 
        _intakeSubsystem.pivotAtSetpoint() && 
        _intakeSubsystem.getIntakePosition() == IntakePosition.SpeakerScore;
    }
}

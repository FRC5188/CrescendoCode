package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class CmdIntakeWaitForIntake extends Command {
    /** Creates a new CmdIntakeWaitForIntake. */
    private Intake _intakeSubsystem;

    public CmdIntakeWaitForIntake(Intake intakeSubsystem) {
        _intakeSubsystem = intakeSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return _intakeSubsystem.pivotAtSetpoint();
    }
}

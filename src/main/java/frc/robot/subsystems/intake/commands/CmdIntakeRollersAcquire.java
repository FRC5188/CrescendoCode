package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/**
 * <STRONG>Purpose: </STRONG>Sets the speed for acquiring a note </p>
 * <STRONG>Subsystem: </STRONG>Intake </p>
 * <STRONG>Status: </STRONG> Working
 */
public class CmdIntakeRollersAcquire extends Command {
    private Intake _intakeSubsystem;

    public CmdIntakeRollersAcquire(Intake intakeSubsystem) {
        _intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        _intakeSubsystem.setRollerMotorSpeedAcquire();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

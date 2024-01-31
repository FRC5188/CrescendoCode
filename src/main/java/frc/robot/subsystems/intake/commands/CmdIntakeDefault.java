package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class CmdIntakeDefault extends Command {
    private Intake _intakeSubsystem;

    public CmdIntakeDefault(Intake intakeSubsystem) {
        _intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        _intakeSubsystem.setIntakePosition(_intakeSubsystem.getIntakePosition());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
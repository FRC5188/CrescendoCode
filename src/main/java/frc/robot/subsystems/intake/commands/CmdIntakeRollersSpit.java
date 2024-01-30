package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class CmdIntakeRollersSpit extends Command {

    private Intake _intakeSubsytem;
    private int _counter;

    public CmdIntakeRollersSpit(Intake intakeSubsystem) {
        _intakeSubsytem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        _intakeSubsytem.setRollerMotorSpeedSpit();
    }

    @Override
    public void execute() {
        _counter++;
    }

    @Override
    public boolean isFinished() {
        return _counter >= 50;
    }
}

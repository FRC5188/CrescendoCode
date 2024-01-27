package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class CmdIntakeWaitForIntake extends Command {

    private Intake _intake;

    public CmdIntakeWaitForIntake(Intake intakeSubsystem) {
        _intake = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
    }

 
    //returns true if finished, false if not 
    // tells us if it is at its set point 
    @Override
    public boolean isFinished() {
        return _intake.pivotAtSetpoint(); 

    }
}

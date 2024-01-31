package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class GrpIntakeAcquireNoteFromPosition extends SequentialCommandGroup {
    private Intake _intakeSubsystem;
    private IntakePosition _intakePosition;
    private int _timeoutInMs;

    public GrpIntakeAcquireNoteFromPosition(Intake intakeSubsystem, IntakePosition intakePosition, int timeoutInMs) {
        addRequirements(intakeSubsystem);

        addCommands(
                new GrpIntakeMoveToPosition(intakeSubsystem, intakePosition),
                new CmdIntakeStartRollersAcquire(intakeSubsystem),
                new CmdIntakeWaitForNote(timeoutInMs, intakeSubsystem),
                new CmdIntakeStopRollers(intakeSubsystem),
                new GrpIntakeMoveToPosition(intakeSubsystem, intakePosition.Stowed));
    }
}

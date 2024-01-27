package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class GrpIntakeAcquireNoteFromPosition extends SequentialCommandGroup {
    private Intake _intakeSubsystem;
    private IntakePosition _intakePosition;
    private int _timeoutInMs;

    public GrpIntakeAcquireNoteFromPosition(Intake intakeSubsystem, IntakePosition intakePosition, int timeoutInMs) {
        _intakeSubsystem = intakeSubsystem;
        _intakePosition = intakePosition;
        _timeoutInMs = timeoutInMs;

        addRequirements(_intakeSubsystem);

        addCommands(
                new GrpIntakeMoveToPosition(intakeSubsystem, intakePosition),
                new CmdIntakeStartRollersAcquire(intakeSubsystem),
                new CmdIntakeWaitForNote(_timeoutInMs, intakeSubsystem),
                new CmdIntakeStopRollers(intakeSubsystem),
                new GrpIntakeMoveToPosition(intakeSubsystem, _intakePosition.Stowed));
    }
}

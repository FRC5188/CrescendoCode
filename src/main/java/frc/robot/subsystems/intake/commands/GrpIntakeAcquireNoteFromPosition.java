package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class GrpIntakeAcquireNoteFromPosition extends SequentialCommandGroup {
    private Intake _intakeSubsystem;
    private IntakePosition _intakePosition;

    public GrpIntakeAcquireNoteFromPosition(Intake intakeSubsystem, IntakePosition intakePosition) {
        _intakeSubsystem = intakeSubsystem;
        _intakePosition = intakePosition;

        addRequirements(_intakeSubsystem);

        addCommands(
                new GrpIntakeMoveToPosition(intakeSubsystem, intakePosition),
                new CmdIntakeStartRollersAcquire(intakeSubsystem),
                new CmdIntakeWaitForNote(0, intakeSubsystem),
                new CmdIntakeStopRollers(intakeSubsystem),
                new GrpIntakeMoveToPosition(intakeSubsystem, _intakePosition.Stowed));
    }
}

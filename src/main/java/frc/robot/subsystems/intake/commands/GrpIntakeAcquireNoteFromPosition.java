package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class GrpIntakeAcquireNoteFromPosition extends SequentialCommandGroup {

    public GrpIntakeAcquireNoteFromPosition(Intake intakeSubsystem, IntakePosition intakePosition, int timeoutInMs) {
        addRequirements(intakeSubsystem);

        addCommands(
                //new GrpIntakeMoveToPosition(intakeSubsystem, intakePosition),
                new CmdIntakeRollersAcquire(intakeSubsystem),
                new CmdIntakeWaitForNote(timeoutInMs, intakeSubsystem),
                new CmdIntakeStopRollers(intakeSubsystem)
                //new GrpIntakeMoveToPosition(intakeSubsystem, IntakePosition.Stowed)
                );
    }
}

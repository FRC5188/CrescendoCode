package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class GrpIntakeAcquireNoteFromSource extends SequentialCommandGroup {

    public GrpIntakeAcquireNoteFromSource(Intake intakeSubsystem, int timeoutInMs) {
        addRequirements(intakeSubsystem);

        addCommands(
                new GrpIntakeAcquireNoteFromPosition(intakeSubsystem, IntakePosition.SourcePickup, timeoutInMs));
    }
}

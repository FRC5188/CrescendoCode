package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class GrpIntakeMoveToPosition extends SequentialCommandGroup {

    public GrpIntakeMoveToPosition(Intake intakeSubsystem, IntakePosition intakePosition) {

        addRequirements(intakeSubsystem);

        addCommands(
                new CmdIntakeSetPosition(intakeSubsystem, intakePosition),
                new CmdIntakeWaitForIntake(intakeSubsystem));
    }
}

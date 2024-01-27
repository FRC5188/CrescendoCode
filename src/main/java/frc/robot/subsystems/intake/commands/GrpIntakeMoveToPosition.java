package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class GrpIntakeMoveToPosition extends SequentialCommandGroup {
    private Intake _intakeSubsystem;
    private IntakePosition _intakePosition;

    public GrpIntakeMoveToPosition(Intake intakeSubsystem, IntakePosition intakePosition) {
        _intakeSubsystem = intakeSubsystem;
        _intakePosition = intakePosition;

        addRequirements(_intakeSubsystem);

        addCommands(
                new CmdIntakeSetPosition(_intakeSubsystem, _intakePosition),
                new CmdIntakeWaitForIntake(_intakeSubsystem));
    }
}

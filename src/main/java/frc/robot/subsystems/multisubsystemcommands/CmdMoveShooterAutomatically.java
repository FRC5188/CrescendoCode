package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.commands.GrpIntakeMoveToPosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

public class CmdMoveShooterAutomatically extends Command {
    private Drive _drive;
    private Shooter _shooterSubsystem;
    private Intake _intakeSubsystem;

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if ((_intakeSubsystem.hasNote()) && (_shooterSubsystem.isAutoShootEnabled())) {
            ShooterZone zone = _shooterSubsystem.getZoneFromRadius(_drive.getRadiusToSpeakerInInches());
            if (zone != _shooterSubsystem.getCurrentZone()) {
                _shooterSubsystem.runShooterForZone(zone);

                if (zone == ShooterZone.Unknown) {
                    new GrpIntakeMoveToPosition(_intakeSubsystem, IntakePosition.Stowed);
                } else {
                    new GrpIntakeMoveToPosition(_intakeSubsystem, IntakePosition.SpeakerScore);
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

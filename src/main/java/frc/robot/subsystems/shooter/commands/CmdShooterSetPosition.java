package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

public class CmdShooterSetPosition extends Command {

    private Shooter _shooterSubsystem;

    public CmdShooterSetPosition(Shooter shooterSubsystem) {
        _shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    ShooterZone shooterZone;

    @Override
    public void initialize() {

        switch (shooterZone) {
            case AMP_SCORE:
                _shooterSubsystem.setTargetPositionAsAngle(ShooterConstants.AMP_SCORE_ANGLE);
                break;
            case SPEAKER_SCORE:
                _shooterSubsystem.setTargetPositionAsAngle(ShooterConstants.SPEAKER_SCORE_ANGLE);
                break;
            case PODIUM:
                _shooterSubsystem.setTargetPositionAsAngle(ShooterConstants.PODIUM_ANGLE);
                break;
            default:
            //I have no clue if anything is supposed to go here.
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

public class CmdShooterSetPosition extends Command {

    private Shooter _shooterSubsystem;

    public CmdShooterSetPosition(Shooter shooterSubsystem) {
        _shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    ShooterZone position;

    @Override
    public void initialize() {
        _shooterSubsystem.setShooterPosition(position);
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

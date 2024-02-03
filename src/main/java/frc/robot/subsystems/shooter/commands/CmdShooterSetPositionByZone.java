package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

public class CmdShooterSetPositionByZone extends Command {

    private Shooter _shooterSubsystem;
    private ShooterZone _zone;

    public CmdShooterSetPositionByZone(Shooter shooterSubsystem, ShooterZone zone) {
        _shooterSubsystem = shooterSubsystem;
        _zone = zone;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        _shooterSubsystem.setShooterPositionWithZone(_zone);
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

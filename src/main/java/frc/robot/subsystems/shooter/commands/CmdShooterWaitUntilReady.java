package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class CmdShooterWaitUntilReady extends Command {
    private Shooter _shooterSubsystem;

    public CmdShooterWaitUntilReady(Shooter shooterSubsystem) {
        _shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        

    }

    @Override
    public boolean isFinished() {
        return _shooterSubsystem.isReady();
    }
}

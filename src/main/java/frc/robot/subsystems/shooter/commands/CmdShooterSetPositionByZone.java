package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

public class CmdShooterSetPositionByZone extends Command {

    private Shooter _shooterSubsystem;
    private ShooterZone _zone;

    /**
     * This redefines the shooterSubsystem and the zone depending on the shooter system
     * @param shooterSubsystem
     * @param zone
     */
    public CmdShooterSetPositionByZone(Shooter shooterSubsystem, ShooterZone zone) {
        _shooterSubsystem = shooterSubsystem;
        _zone = zone;

        addRequirements(shooterSubsystem);
    }

    @Override
    /**
     * It sets the shooter in the right position depending on what zone the robot is in
     */
    public void initialize() {
        _shooterSubsystem.setShooterPositionWithZone(_zone);
    }

    @Override
    /**
     * waste of lines
     */
    public void execute() {

    }

    @Override
    /**
     * waste of lines
     */
    public void end(boolean interrupted) {

    }

    @Override
    /**
     * It stops a series of code
     */
    public boolean isFinished() {
        return true;
    }
}

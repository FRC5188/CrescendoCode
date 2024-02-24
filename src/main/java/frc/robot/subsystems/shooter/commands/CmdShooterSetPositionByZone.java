package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

public class CmdShooterSetPositionByZone extends Command {

    private Shooter _shooterSubsystem;
    private ShooterZone _zone;

    public CmdShooterSetPositionByZone(Shooter shooterSubsystem, ShooterZone zone) {
        this._shooterSubsystem = shooterSubsystem;
        this._zone = zone;

        addRequirements(shooterSubsystem);
    }

    //<STRONG>Purpose:</STRONG> Update the setpoint of the PID based on the zone that our robot is in.
    @Override
    public void initialize() {
        _shooterSubsystem.setShooterPositionWithZone(_zone);
    }

    @Override
    public boolean isFinished() {
        return true; // Only run once.
    }
}

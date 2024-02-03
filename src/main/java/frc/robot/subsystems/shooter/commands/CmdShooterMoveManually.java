package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class CmdShooterMoveManually extends Command {

    private Shooter _shooterSubsystem;
    private double _changeAmount;

    public CmdShooterMoveManually(Shooter shooterSubsystem, double changeAmount) {
        _shooterSubsystem = shooterSubsystem;
        _changeAmount = changeAmount;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        double setpoint = _shooterSubsystem.getCurrentZone().getShooterAngle() + _changeAmount;
        System.out.println ("Changing Shooter Position from " + _shooterSubsystem.getCurrentZone().getShooterAngle() + " to " + setpoint);
        _shooterSubsystem.setTargetPositionAsAngle(setpoint);

    }
    @Override
    public boolean isFinished() {
        return true;
    }
    
}

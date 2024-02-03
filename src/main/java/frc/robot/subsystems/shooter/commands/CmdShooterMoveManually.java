package frc.robot.subsystems.shooter.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class CmdShooterMoveManually extends Command {

    private Shooter _shooterSubsystem;
    private DoubleSupplier _changeAmount;

    public CmdShooterMoveManually(Shooter shooterSubsystem, DoubleSupplier changeAmount) {
        _shooterSubsystem = shooterSubsystem;
        _changeAmount = changeAmount;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        double setpoint = _shooterSubsystem.getCurrentPositionInDegrees() + _changeAmount.getAsDouble();
        System.out.println ("Changing Shooter Position from" + _shooterSubsystem.getCurrentPositionInDegrees() + "to" + setpoint);
        _shooterSubsystem.setTargetPositionAsAngle(setpoint);

    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
    
}

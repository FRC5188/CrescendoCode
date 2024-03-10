package frc.robot.subsystems.climber.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class CmdClimberMove extends Command {
    private Climber _climberSubsystem;
    private DoubleSupplier _leftSpeed;
    private DoubleSupplier _rightSpeed;

    public CmdClimberMove(Climber climberSubsystem, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
        _climberSubsystem = climberSubsystem;
        _leftSpeed = leftSpeed;
        _rightSpeed = rightSpeed;
    }

    @Override
    public void execute() {
        if (_climberSubsystem.canMove()) {
            _climberSubsystem.setClimberLeftSpeed(_leftSpeed.getAsDouble());
            _climberSubsystem.setClimberRightSpeed(_rightSpeed.getAsDouble());
        } else {
            _climberSubsystem.setClimberLeftSpeed(0);
            _climberSubsystem.setClimberRightSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        _climberSubsystem.setClimberLeftSpeed(0);
        _climberSubsystem.setClimberRightSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

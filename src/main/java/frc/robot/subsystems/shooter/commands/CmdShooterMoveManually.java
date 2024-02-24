package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

/** 
 * <STRONG>Purpose:</STRONG> Move the pivot by a specified delta added to the current angle of the pivot in degrees. It should be noted that if you want to decrease the 
 * position then you should add negative sign.</p>
 * <STRONG>Status:</STRONG> Working
*/
public class CmdShooterMoveManually extends Command {

    private Shooter _shooterSubsystem;
    private double _changeAmount;

    public CmdShooterMoveManually(Shooter shooterSubsystem, double changeAmount) {
        this._shooterSubsystem = shooterSubsystem;
        this._changeAmount = changeAmount;

        addRequirements(shooterSubsystem);
    }

    /**
     * <STRONG>Purpose: </STRONG> We add or subtract the inputted change to the current angle of the pivot based on 
     * angle that's based on the zone we're in. </p>
     */
    @Override
    public void initialize() {
        double setpoint = _shooterSubsystem.getCurrentZone().getShooterAngle() + _changeAmount;
        System.out.println ("Changing Shooter Position from " + _shooterSubsystem.getCurrentZone().getShooterAngle() + " to " + setpoint);
        _shooterSubsystem.setTargetPositionAsAngle(setpoint);

    }
    
    @Override
    public boolean isFinished() {
        return true; // Only run once.
    }
}

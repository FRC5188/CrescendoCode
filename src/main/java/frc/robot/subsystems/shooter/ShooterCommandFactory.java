package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

public class ShooterCommandFactory {
    private Shooter _shooter;

    public ShooterCommandFactory(Shooter shooter) {
        this._shooter = shooter;
    }

    /** Sets the angle of the Shooter. */
    public Command setAngle(double angle) {
        return new InstantCommand(() -> 
            this._shooter.setTargetPositionAsAngle(angle), 
            this._shooter);
    }

    public Command adjustAngle(double angle) {
        return new InstantCommand(() -> 
            this._shooter.setTargetPositionAsAngle(_shooter.getCurrentZone().getShooterAngle() + angle), 
            this._shooter);
    }

    public Command runFlywheelsForZone(Shooter.ShooterZone zone) {
        return new StartEndCommand(
        // starts flywheels at beginning of command
        () -> 
        this._shooter.setFlywheelSpeed(zone.getLeftFlywheelSpeed()),
        // stops flywhells at end of command
        () -> 
        this._shooter.stopFlywheels(),
        this._shooter);
        }


    public Command runPIDs() {
        return new Command() {
          @Override
          public void execute() {
              _shooter.runAnglePid();
          }  
          @Override
              public boolean isFinished() {
                  return false;
              }
        };
    }

    public Command runForZone(ShooterZone zone) {
        return new InstantCommand (() ->
        this._shooter.runShooterForZone(zone),
        this._shooter
        );

    }

    public Command setAutoShootEnabled(boolean enabled) {
        return new InstantCommand(() ->
        this._shooter.setAutoShootEnabled(enabled),
        this._shooter);
    }

    public Command setPositionByZone(Shooter.ShooterZone zone) {
        return new InstantCommand (() ->
        this._shooter.setShooterPositionWithZone(zone),
        this._shooter
        );
    }
}

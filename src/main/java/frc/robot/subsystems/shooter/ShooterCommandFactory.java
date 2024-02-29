package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

class ShooterCommandFactory {
    private Shooter _shooter;

    protected ShooterCommandFactory(Shooter shooter) {
        this._shooter = shooter;
    }

    /** Sets the angle of the Shooter. */
    protected Command setAngle(double angle) {
        return new InstantCommand(() -> 
            _shooter.setTargetPositionAsAngle(angle), 
            _shooter);
    }

    protected Command adjustAngle(double angle) {
        return new InstantCommand(() -> 
            _shooter.setTargetPositionAsAngle(_shooter.getCurrentZone().getShooterAngle() + angle), 
            _shooter);
    }

    protected Command runFlywheels() {
        return new StartEndCommand(() ->
        null, null, null)
    }
}
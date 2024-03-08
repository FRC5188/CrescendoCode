package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

public class ShooterCommandFactory {
    private Shooter _shooter;

    /**
     * Use this to access shooter commands
     * 
     * @param shooter
     */
    public ShooterCommandFactory(Shooter shooter) {
        this._shooter = shooter;
    }

    /**
     * Sets the angle of the shooter in degrees
     * 
     * @param angle andle in degrees
     * @return Command to set shooter angle
     */
    public Command setAngle(double angle) {
        return new InstantCommand(() -> this._shooter.setTargetPositionAsAngle(angle),
                this._shooter);
    }

    /**
     * Adjust the current angle of the shooter up or down.
     * This is based on the current zone's setpoint, not the
     * specific angle we are current at.
     * 
     * @param angle to increment or decrement the shooter by
     * @return command to adjust shooter angle
     */
    public Command adjustAngle(double angle) {
        return new InstantCommand(
                () -> this._shooter.adjustShooterAngle(angle),
                this._shooter);
    }

    /**
     * Run the flywheels on the shooter for the supplied zone. The command stops the 
     * fly wheels when the command ends.
     * @param zone
     * @return command to run the flywheels
     */
    public Command runFlywheelsForZone(ShooterZone zone) {
        // 
        return new StartEndCommand(
                // starts flywheels at beginning of command
                () -> this._shooter.setFlywheelSpeedWithZone(zone),
                // stops flywheels at end of command
                () -> this._shooter.stopFlywheels(),
                this._shooter);
    }

    /**
     * Command to run the angle PID of the shooter. This command will never finish
     * @return command to run the shooter angle PID
     */
    public Command runAnglePID() {
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

    /**
     * Set the shooter angle and flywheel speed for a given zone. A zone
     * must be one of our predefined zones such as subwoofer, podium, etc
     * @param zone shooter zone to run shooter for
     * @return command to run the shooter for a zone
     */
    public Command runForZone(ShooterZone zone) {
        return new InstantCommand(() -> this._shooter.runShooterForZone(zone),
                this._shooter);
    }

    /**
     * Tells the shooter subsystem to enable or disable the autoshoot functionality.
     * 
     * @param enabled true to enable autoshoot and false to disable
     * @return a new command to enable or disable autoshoot
     */
    public Command setAutoShootEnabled(boolean enabled) {
        return new InstantCommand(() -> this._shooter.setAutoShootEnabled(enabled),
                this._shooter);
    }

    /**
     * Sets the shooter angle based on a zone. this does NOT run the flywheels.
     * The zone must be a predefined Shooter.ShooterZone type such as podium, speaker, etc
     * @param zone a Shooter.ShooterZone type
     * @return a command to set the shooter position
     */
    public Command setPositionByZone(ShooterZone zone) {
        return new InstantCommand(() -> this._shooter.setShooterPositionWithZone(zone),
                this._shooter);
    }
}

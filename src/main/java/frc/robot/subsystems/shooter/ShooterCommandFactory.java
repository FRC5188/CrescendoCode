package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;
// TODO: Add a true/false that will disable the PID on each componenet of the shooter. This will be useful for testing and for when we're not using the shooter.

public class ShooterCommandFactory {
    private Shooter _shooter;

    /**
     * Should be reference in the Shooter.java subsystem serving as a method of creating command for shooting.
     */
    public ShooterCommandFactory(Shooter shooter) {
        this._shooter = shooter;
    }

    /**
     * Sets the angle of the shooter in degrees. Note that this will manually update the setpoint of the PID for the given angle, but the 
     * robot will still think it's in the same zone.
     * @param angle Angle in degrees that you want to set the shooter's pivot to.
     * @return Command which moves the pivot to the specified angle.
     */
    public Command setAngle(double angle) {
        return new InstantCommand(() -> 
            _shooter.setTargetPositionAsAngle(angle), 
            _shooter);
    }

    /**
     * @param zone The zone that the robot is in.
     * @return Command which will set the angle of the pivot to the angle set by the zone.
     */
    public Command setAngle(ShooterZone zone) {
        return setAngle(zone.getShooterAngle());
    }

    /**
     * @param angle The delta that you want to adjust the angle set by the zone to in degrees. Positive is toward the sky.
     * @return Command which will add the delta angle to the angle set by the zone.
     */
    public Command adjustAngle(double angle) {
        return setAngle(angle + _shooter.getCurrentZone().getShooterAngle());
    }

    /**
     * @return Command which should be continuously run while the robot is running that keeps the pivot and the correct angle.
     */
    public Command runPID() {
        return new RunCommand(_shooter::runAnglePid, _shooter);
    }

    /**
     * @param speed The speed in RPM (Rotations Per Minute) that you want both flywheels to run at.
     * @return Command which sets the flywheels to the specified speed.
     */
    public Command runFlywheel(double speed) {
        return new InstantCommand(
            () -> _shooter.setFlywheelSpeed(speed),
            _shooter
        );
    }

    /**
     * @param zone The zone that the robot is in.
     * @return Command which given the zone the robot is in will set the angle of th pivot and of the flywheels running in parallel.
     */
    public Command runForZone(ShooterZone zone) {
        return new InstantCommand(
            () -> _shooter.runShooterForZone(zone),
            _shooter
        );
    }

    /**
     * @param enabled True if you want to use autoshooting, false if you want to disable it.
     * @return Command which updates the boolean that determines whether we'll use autoshooting or not.
     */
    public Command setAutoShooting(boolean enabled) {
        return new InstantCommand(
            () -> _shooter.setAutoShootEnabled(enabled),
            _shooter
        );
    }

    /**
     * @param _intake The Inktake Subystem.
     * @return Command which will end whenever both Shooter and Intake are ready.
     */
    public Command waitUntilReady(Intake _intake) {
        return new Command() {
            @Override
            public boolean isFinished() {
                return _shooter.isReady() && 
                _intake.pivotAtSetpoint() && 
                _intake.getIntakePosition() == Intake.IntakePosition.SpeakerScore; // TODO: We'll have to add a deadband here since the intake won't exactly be in the right place.
            }
        };
    }
}
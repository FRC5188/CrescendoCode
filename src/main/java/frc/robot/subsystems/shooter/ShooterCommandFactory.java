package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class ShooterCommandFactory {
    private Shooter _shooter;
    private Shooter _shooterSubsystem;
    private Intake _intakeSubsystem;
    private ShooterZone _zone;
    private boolean _enabled;

    protected ShooterCommandFactory(Shooter shooter, Shooter shooterSubsystem, Intake intakeSubsystem, ShooterZone zone, boolean _enabled) {
        this._shooter = shooter;
        this._shooterSubsystem = shooterSubsystem;
        this._intakeSubsystem = intakeSubsystem;
        this._zone = zone;
        this._enabled = enabled;
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

    protected Command runFlywheelsForZone() {
        return StartEndCommand(() ->
        // TODO
        )
    }

    protected Command runPIDs() {
        return new InstantCommand(() ->
        _shooterSubsystem.runAnglePid(),
        _shooterSubsystem
        );
    }

    protected Command runForZone(ShooterZone zone) {
        return new InstantCommand (() ->
        _shooterSubsystem.runShooterForZone(_zone),
        _shooterSubsystem
        );

    }

    protected Command setAutoShootEnabled(boolean enabled) {
        return new InstantCommand (() ->
        _shooterSubsystem.setAutoShootEnabled(_enabled),
        _shooterSubsystem
        );
    }

    protected Command setPositionByZone() {
        return new InstantCommand (() ->
        _shooterSubsystem.setShooterPositionWithZone(_zone),
        _shooterSubsystem
        );
    }

     protected Command waitUntilReady() {
        return new Command() {
            @Override
            public boolean isFinished() {
                return _shooterSubsystem.isReady() && 
        _intakeSubsystem.pivotAtSetpoint() && 
        _intakeSubsystem.getIntakePosition() == IntakePosition.SpeakerScore;
            }
        };
    }
}
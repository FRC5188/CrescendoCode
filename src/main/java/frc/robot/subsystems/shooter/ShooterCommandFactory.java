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
    private ShooterZone _zone;
    private Intake _intakeSubsystem;
    private boolean _enabled;

    public ShooterCommandFactory(Shooter shooter, Shooter shooterSubsystem, ShooterZone zone, Intake intakeSubsystem, boolean enabled) {
        this._shooter = shooter;
        this._shooterSubsystem = shooterSubsystem;
        this._zone = zone;
        this._intakeSubsystem = intakeSubsystem;
        this._enabled = enabled;
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

    // HELP
    // public Command runFlywheelsForZone(double speedInRPM) {
    //     return StartEndCommand(
    //     this._shooterSubsystem.setFlywheelSpeed(_zone.getLeftFlywheelSpeed()),
    //     () -> {
    //         this._shooterSubsystem.stopFlywheels();
    //     }, this._shooterSubsystem)

    // }

    public Command runPIDs() {
        return new InstantCommand(
        this._shooterSubsystem::runAnglePid,
        this._shooterSubsystem
        );
    }

    public Command runForZone(ShooterZone zone) {
        return new InstantCommand (() ->
        this._shooterSubsystem.runShooterForZone(_zone),
        this._shooterSubsystem
        );

    }

    public Command setAutoShootEnabled(boolean enabled) {
        return new InstantCommand(() ->
        this._shooterSubsystem.setAutoShootEnabled(_enabled),
        this._shooterSubsystem);
    }

    public Command setPositionByZone() {
        return new InstantCommand (() ->
        this._shooterSubsystem.setShooterPositionWithZone(_zone),
        this._shooterSubsystem
        );
    }

   // HELP
    //  public Command waitUntilReady() {
    //     return new Command() {
    //         @Override
    //         public boolean isFinished() {
    //             return this._shooterSubsystem.isReady() && 
    //             this._intakeSubsystem.pivotAtSetpoint() && 
    //             this._intakeSubsystem.getIntakePosition() == IntakePosition.SpeakerScore;
    //         }
    //     };
    // }
}
package frc.robot.subsystems.multisubsystemcommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

public class CmdAdjustShooterAutomatically extends Command {
    private Drive _drive;
    private Shooter _shooterSubsystem;
    private Intake _intakeSubsystem;
    private boolean _prevAutoshootState;

    public CmdAdjustShooterAutomatically(Drive drive, Shooter shooter, Intake intake) {
        _drive = drive;
        _shooterSubsystem = shooter;
        _intakeSubsystem = intake;
    }

    @Override
    public void initialize() {
        _prevAutoshootState = true;
    }

    @Override
    public void execute() {
        // Only run things if autoshoot is enabled and we have a note
        if (_shooterSubsystem.isAutoShootEnabled()) {
            // update this flag so we can properly disable the flywheels if we turn
            // autoshoot off
            _prevAutoshootState = true;

            // If we have a note, then start running the shooter according to the radius to
            // speaker
            if (_intakeSubsystem.hasNote()) {
                double radius = _drive.getRadiusToSpeakerInMeters();
                _shooterSubsystem.runShooterForRadius(radius);
                Logger.recordOutput("Shooter/RadiusToSpeaker", radius);
            } else {
                // We aren't holding a note, so stay in unknown for now
                _shooterSubsystem.runShooterForZone(ShooterZone.Unknown);
            }
        } else {
            // We want to use manual control, so put the shooter to unknown
            // We only want to do this once, right when we transition,
            // otherwise this command will try to override manual shooting
            if (_prevAutoshootState) {
                // We just flipped the switch, so set to unknown
                _shooterSubsystem.runShooterForZone(ShooterZone.Unknown);
                // update this flag so we don't set to unknown repeatedly
                _prevAutoshootState = false;
            }
        }
        _shooterSubsystem.runAnglePID();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

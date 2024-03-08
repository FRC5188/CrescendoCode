package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;

public class CmdAdjustShooterAutomatically extends Command {
    private Drive _drive;
    private Shooter _shooterSubsystem;
    private Intake _intakeSubsystem;

    public CmdAdjustShooterAutomatically(Drive drive, Shooter shooter, Intake intake) {
        _drive = drive;
        _shooterSubsystem = shooter;
        _intakeSubsystem = intake;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (_shooterSubsystem.isAutoShootEnabled()) {
            if (_intakeSubsystem.hasNote()) {
                // TODO: Maybe add in a tolerance for when we straddle zones so we don't go back
                // and forth
                ShooterZone zone = _shooterSubsystem.getZoneFromRadius(_drive.getRadiusToSpeakerInMeters());
                if (zone != _shooterSubsystem.getCurrentZone()) {

                    // We actually want to shoot
                    _shooterSubsystem.runShooterForZone(zone);
                }
            } else {
                if (_shooterSubsystem.getCurrentZone() != ShooterZone.Unknown) {
                    // We don't have a note, so enter our safe mode, which is kept in Unknown
                    _shooterSubsystem.runShooterForZone(ShooterZone.Unknown);
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

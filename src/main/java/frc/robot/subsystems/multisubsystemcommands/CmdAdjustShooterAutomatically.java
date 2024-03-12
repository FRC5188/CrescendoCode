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
                double radius = _drive.getRadiusToSpeakerInMeters();
                // We actually want to shoot
                Logger.recordOutput("Shooter/RadiusSpeaker", radius);
                _shooterSubsystem.runShooterForRadius(radius);
            } else {
                    _shooterSubsystem.runShooterForZone(ShooterZone.Unknown);
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

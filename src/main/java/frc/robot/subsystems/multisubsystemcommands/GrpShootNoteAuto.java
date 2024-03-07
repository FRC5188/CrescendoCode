package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;
import frc.robot.subsystems.shooter.commands.CmdShooterRunShooterForZone;
import frc.robot.subsystems.shooter.commands.CmdShooterSetAutoshootEnabled;
import frc.robot.subsystems.shooter.commands.CmdShooterWaitUntilReady;

// This command should use current robot location to detect zone, but is untested.
public class GrpShootNoteAuto extends SequentialCommandGroup {
    ShooterZone zone;

    public GrpShootNoteAuto(Intake intakeSubsystem, Shooter shooterSubsystem, Drive driveSubsystem) {

        addRequirements(intakeSubsystem, shooterSubsystem);

        zone = shooterSubsystem.getZoneFromRadius(driveSubsystem.getRadiusToSpeakerInMeters());
    

        System.out.print("Zone: ");
        System.out.println(zone);
        

        addCommands( 
                new CmdShooterSetAutoshootEnabled(shooterSubsystem, false),
                intakeSubsystem.buildCommand().setPosition(IntakePosition.Stowed),
                new CmdShooterRunShooterForZone(shooterSubsystem, zone),
                new CmdShooterWaitUntilReady(shooterSubsystem, intakeSubsystem),
                intakeSubsystem.buildCommand().spit(IntakeConstants.INTAKE_SPIT_TIME),                
                new CmdShooterRunShooterForZone(shooterSubsystem, ShooterZone.Unknown));
                intakeSubsystem.buildCommand().setPosition(IntakePosition.Stowed);
    }

}

package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.commands.CmdIntakeRunPID;
import frc.robot.subsystems.intake.commands.CmdIntakeSetPosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterZone;
import frc.robot.subsystems.shooter.commands.CmdShooterRunPids;
import frc.robot.subsystems.shooter.commands.CmdShooterRunShooterForZone;

public class GrpSetUp extends ParallelCommandGroup {
  // This group gets called whenever we start the robot in auto or teleop
  // Put anything running PIDs or setting up initial states here
  public GrpSetUp(Drive drive, Shooter shooter, Intake intake) {
    addCommands(
      //new CmdRunShooterAutomatically(drive, shooter, intake),
      new CmdIntakeSetPosition(intake, IntakePosition.Stowed),
      new CmdShooterRunShooterForZone(shooter, ShooterZone.Unknown),
      new CmdIntakeRunPID(intake),
      new CmdShooterRunPids(shooter)
    );
  }
}

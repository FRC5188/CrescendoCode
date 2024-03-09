package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class CmdShoot extends Command {
  private Shooter _shooterSubsystem;
  private Intake _intakeSubsystem;

  public CmdShoot(Shooter shooterSubsystem, Intake intakeSubsystem, double timeoutSeconds) {
    _shooterSubsystem = shooterSubsystem;
    _intakeSubsystem = intakeSubsystem;

    this.withTimeout(timeoutSeconds);
  }

  @Override
  public void initialize() {
    _shooterSubsystem.setFeederMotorShootSpeed();
    _intakeSubsystem.setRollerMotorSpeedSpit();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    _shooterSubsystem.setFeederMotorPickupSpeed();
    _intakeSubsystem.stopRollerMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

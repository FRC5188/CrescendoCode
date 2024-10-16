// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.*;

public class CmdIntakeWaitForNote extends Command {
  /** Creates a new CmdIntakeWaitForNote. */
  private int _countdownInCycles;
  private int _timeoutInMS;
  private boolean _commandTimesOut;
  private Intake _intakeSubsystem;

  /**
   * Waits for the intake to acquire a note or for the timeout to be reached.
   * If a timeout of <= 0 is given, this command will not time out.
   * 
   * @param timeoutInMs Time waited in milliseconds for the intake to acquire a
   *                    note.
   */

  public CmdIntakeWaitForNote(int timeoutInMs, Intake intakeSubsystem) {

    _intakeSubsystem = intakeSubsystem;
    _timeoutInMS = timeoutInMs;

    // If a timeout of <=0 is given, this command will never time out.
    _commandTimesOut = !(timeoutInMs <= 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Current state of the countdown in cycles. 20 ms per robot cycle.
    _countdownInCycles = (int) Math.ceil(_timeoutInMS / 20.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _countdownInCycles--;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (_commandTimesOut && _countdownInCycles <= 0) || _intakeSubsystem.hasNote();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class CmdAcquireNoteFor extends Command {
  private int _cyclesLeft;
  private int _timeToRunMS;
  private Intake _intake;
  private double _speed;

  /**
   * Runs the intake rollers for a certain amount of certain. Turns off the rollers after this amount of time
   * regardless of if it actually got a note. 
   * @param timeToRunMS
   * @param intake
   */
  public CmdAcquireNoteFor(int timeToRunMS, Intake intake, double speed) {
    this._intake = intake;
    this._timeToRunMS = timeToRunMS;
    this._speed = speed;

    this.addRequirements(this._intake);
  }

  @Override
  public void initialize() {
    // Sets the amount of cycles that you want to run the command for based on the time given.
    this._cyclesLeft = (int) Math.ceil(this._timeToRunMS / 20.0);
    this._intake.setRollerMotorSpeed(_speed);
  }

  @Override
  public void execute() {
      this._cyclesLeft--;
    }

  @Override
  public void end(boolean interrupted) {
    // Once we're done we stop the rollers. 
    this._intake.stopRollerMotor();
  }

  @Override
  public boolean isFinished() {
    return this._cyclesLeft <= 0;
  }
}

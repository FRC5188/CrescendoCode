// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/** 
 * Command which will set intake to aquire for a set amount of time as given in the parameter. After that time has ran it'll
 * stop the intake
 */
public class CmdAquireNoteFor extends Command {
  private int _cyclesLeft;
  private int _timeToRun;
  private Intake _intake;

  public CmdAquireNoteFor(int timeToRun, Intake intake) {
    this._intake = intake;
    this._timeToRun = timeToRun;

    this.addRequirements(this._intake);
  }

  @Override
  public void initialize() {
    // Sets the amount of cycles that you want to run the command for based on the time given.
    this._cyclesLeft = (int) Math.ceil(this._timeToRun / 20.0);
    this._intake.setRollerMotorSpeedAcquire();
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

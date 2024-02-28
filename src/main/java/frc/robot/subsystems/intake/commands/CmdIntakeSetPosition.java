// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class CmdIntakeSetPosition extends Command {
    private Intake _intakeSubsystem;
    private IntakePosition _intakePosition;

    /**
     * Sets the intake position based on given intake position like amp, stowed, ground pickup, etc.
     * Use with the button.whileTrue function
     * @param intakeSubsystem
     * @param intakePosition
     */
    public CmdIntakeSetPosition(Intake intakeSubsystem, IntakePosition intakePosition) {
        this._intakeSubsystem = intakeSubsystem;
        this._intakePosition = intakePosition;

        addRequirements(intakeSubsystem);
    }

    // // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _intakeSubsystem.setIntakePosition(this._intakePosition);
        if (_intakePosition == IntakePosition.Stowed) {
            _intakeSubsystem.stopRollerMotor();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}

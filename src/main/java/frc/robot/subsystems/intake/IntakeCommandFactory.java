package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.intake.Intake.IntakePosition;

class ShooterCommandFactory {
    private Intake _intake;
    private Intake _intakeSubsystem;
    private IntakePosition _intakePosition;

    protected void IntakeCommandFactory(Intake intake, Intake intakeSubsystem) {
        this._intake = intake;
        this._intakeSubsystem = intakeSubsystem;
    }
    // sets the roller speed to acquire
    protected Command rollerAcquire() {
        return new InstantCommand(() ->
        _intake.setRollerMotorSpeedAcquire(),
        _intake);
    }

    // sets the roller speed to spit
    protected Command rollerSpit() {
        return new InstantCommand(() ->
        _intake.setRollerMotorSpeedSpit(),
        _intake);

    }

    // runs the PID
    protected Command runPID() {
        return new RunCommand(() ->
        _intakeSubsystem.runPivotPID(),
        _intakeSubsystem);
    }

    // sets intake position to a given IntakePosition
    protected Command setPosition() {
        return new InstantCommand(() ->
        _intake.setIntakePosition(_intakePosition),
        _intake);
    }

    // stops the roller motors
    protected Command stopRollers() {
        return new InstantCommand(() ->
        _intakeSubsystem.stopRollerMotor(),
        _intakeSubsystem);
    }
    
    // waits until pivot is at its setpoint
    protected Command waitForIntake() {
        return new Command() {
            @Override
            public boolean isFinished() {
                return _intakeSubsystem.pivotAtSetpoint();
            }
        };
    }

    protected Command waitForNote() {
        return new Command() {
            //TODO
            }
        }
    // 
}
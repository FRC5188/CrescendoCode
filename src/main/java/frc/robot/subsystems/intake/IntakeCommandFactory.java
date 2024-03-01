package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class IntakeCommandFactory {
    private Intake _intake;

    public IntakeCommandFactory(Intake intake) {
        this._intake = intake;
    }

    public Command aquire() {
        return new InstantCommand(
            this._intake::setRollerMotorSpeedAcquire,
            this._intake);
    }

    public Command spit() {
        final double TIME_RUN_SECONDS = 1.0;

        return new StartEndCommand(
         this._intake::setRollerMotorSpeedSpit,
         () -> {
                this._intake.stopRollerMotor();
                this._intake.resetHasNote();

         }, 
         this._intake).withTimeout(TIME_RUN_SECONDS);
    }

    public Command runPID() {
        return new RunCommand(
            this._intake::runPivotPID, this._intake);
    }

    public Command setPosition(Intake.IntakePosition position) {
        return new InstantCommand(
            () -> {
                this._intake.setIntakePosition(position);
                if (position == Intake.IntakePosition.Stowed) {
                    this._intake.stopRollerMotor();
                }
            }, this._intake);
    }

    public Command stop(){
        return new InstantCommand(
            this._intake::stopRollerMotor, this._intake);
    }

    public Command waitForIntake() {
        return new RunCommand(() -> {}, this._intake).until(() -> this._intake.pivotAtSetpoint());
    }

    public Command waitForNote() {
        final double TIME_RUN_SECONDS = 0.1;

        return new StartEndCommand(
         this._intake::setRollerMotorSpeedSpit,
         () -> {
                this._intake.stopRollerMotor();
                this._intake.resetHasNote();

         }, this._intake)
            .withTimeout(TIME_RUN_SECONDS)
            .until(() -> this._intake.hasNote());
    }

    public Command pickUpNoteFrom(Intake.IntakePosition position) {
        return this.setPosition(position)
            .alongWith(this.aquire())
            .beforeStarting(this.waitForNote())
            .beforeStarting(this.stop());
    }

    public Command pickUpFromGround() {
        return this.pickUpNoteFrom(Intake.IntakePosition.GroundPickup);
    }

    public Command pickUpFromSource() {
        return this.pickUpNoteFrom(Intake.IntakePosition.SourcePickup);
    }
}

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.commands.CmdIntakeWaitForNote;
import frc.robot.subsystems.intake.commands.CmdAquireNoteFor;

public class IntakeCommandFactory {
    private Intake _intake;

    public IntakeCommandFactory(Intake intake) {
        this._intake = intake;
    }

    /** Turns ON the ROLLERS in the direction toward the robot. Doesn't automatically turn off. */
    public Command aquire() {
        return new InstantCommand(
            this._intake::setRollerMotorSpeedAcquire,
            this._intake);
    }

    /** Turns ON the ROLLERS away from the robot. Doesn't automatically turn off. */
    public Command spit(double timeSeconds) {

        return new StartEndCommand(
         this._intake::setRollerMotorSpeedSpit,
         () -> {
                this._intake.stopRollerMotor();
                this._intake.resetHasNote();

         }, 
         this._intake).withTimeout(timeSeconds);
    }

    /***
     * Returns a new Command object where the execute calls _intake.runPivotPID()
     * and the isFinished is always false. This command does not require the subsystem.
     * 
     * @return a new Command
     */
    public Command runPID() {
        return new Command() {
            @Override
            public void execute() {
                _intake.runPivotPID();
            }
            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    /** Sets the positions of the PID for the Intake. */
    public Command setPosition(Intake.IntakePosition position) {
        return new InstantCommand(
            () -> {
                this._intake.setIntakePosition(position);
                // We'll turn OFF the ROLLERS if we're stowing.
                if (position == Intake.IntakePosition.Stowed) {
                    this._intake.stopRollerMotor();
                }
            }, this._intake);
    }

    /** Turns OFF the ROLLERS. */
    public Command stop(){
        return new InstantCommand(
            this._intake::stopRollerMotor, this._intake);
    }

    /** Will SET THE POSITION for the intake then as it moves to the position will turn ON the ROLLERS.
     * Then it'll wait until it's been given a note based on current cutoff, run for an additional 150ms, then stow.
     * @param position
     * @return
     */
    public Command pickUpNoteFrom(Intake.IntakePosition position) {
        return this.setPosition(position)
            .andThen(this.aquire())
            .andThen(new CmdIntakeWaitForNote(0, this._intake))
            .andThen(new CmdAquireNoteFor(150, _intake))
            .andThen(this.setPosition(IntakePosition.Stowed));
    }

    public Command pickUpFromGround() {
        return this.pickUpNoteFrom(Intake.IntakePosition.GroundPickup);
    }

    public Command pickUpFromSource() {
        return this.pickUpNoteFrom(Intake.IntakePosition.SourcePickup);
    }

    public Command aquire(int timeMS) {
        return new CmdAquireNoteFor(timeMS, this._intake);
    }
}

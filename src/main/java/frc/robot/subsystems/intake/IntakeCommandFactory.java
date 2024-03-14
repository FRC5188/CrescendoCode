package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.commands.CmdIntakeWaitForNote;
import frc.robot.subsystems.intake.commands.CmdAcquireNoteFor;

public class IntakeCommandFactory {
    private Intake _intake;

    /**
     * Use this to access commands to run the intake
     * @param intake {@link Intake}
     */
    public IntakeCommandFactory(Intake intake) {
        this._intake = intake;
    }


     /**
      * Turns ON the ROLLERS in the direction toward the robot. Doesn't automatically turn off.
      * @return
      */
    public Command acquire() {
        return new InstantCommand(
            this._intake::setRollerMotorSpeedAcquire,
            this._intake);
    }

    /**
     *  Turns ON the ROLLERS away from the robot. Will turn off the rollers after
     * the timeout in seconds.
     * @param timeSeconds time to wait until turning off the rollers in seconds
     * @return
     */
    public Command spit(double timeSeconds) {
        return new StartEndCommand(
         () -> {
            this._intake.setRollerMotorSpeedSpit();
            this._intake.setFeederMotorShootSpeed();
         },
         () -> {
                this._intake.stopRollerMotor();
                this._intake.setFeederMotorPickupSpeed();
                this._intake.resetHasNote();

         }, 
         this._intake).withTimeout(timeSeconds);
    }

    /**
     * Sets the positions of the PID for the Intake. positon should be a 
     * {@link Intake.IntakePosition}
     * @param position {@link}
     * @return
     */
    public Command setPosition(IntakePosition position) {
        return new InstantCommand(
            () -> {
                this._intake.setIntakePosition(position);
                // We'll turn OFF the ROLLERS if we're stowing.
                if (position == IntakePosition.Stowed) {
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
     * @param position {@link Intake.IntakePosition}
     * @return
     */
    public Command pickUpNoteFrom(IntakePosition position) {
        return this.setPosition(position)
            .andThen(this.acquire())
            .andThen(new CmdIntakeWaitForNote(0, this._intake))
            .andThen(new CmdAcquireNoteFor(1000, _intake, IntakeConstants.INTAKE_ACQUIRE_SPEED))
            .andThen(this.setPosition(IntakePosition.Stowed));
        // return new InstantCommand(this.setPosition(position)).andThen(
        //     new RunCommand(
        //     this._intake.r, null))
    }

    /**
     * Puts the intake to {@link Intake.IntakePosition.GroundPickup}, runs the rollers, picks up note,
     * turns off rollers, and puts the intake to {@link Intake.IntakePosition.Stowed}.
     * 
     * This command calles the {@link IntakeCommandFactory.pickUpNoteFrom}
     * @return
     */
    public Command pickUpFromGround() {
        return this.pickUpNoteFrom(IntakePosition.GroundPickup);
    }

    /**
     * Run the intake to grab a note. Runs for a preset amount of time.
     * Does not use any auto detection.
     * @param timeMS time in ms to try to grab the note
     * @return
     */
    public Command acquire(int timeMS) {
        return new CmdAcquireNoteFor(timeMS, this._intake, IntakeConstants.INTAKE_ACQUIRE_SPEED);
    }
}

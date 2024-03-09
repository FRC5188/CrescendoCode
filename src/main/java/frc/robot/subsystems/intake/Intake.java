package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged; checkstyle says this is redunant
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Tunable;

public class Intake extends SubsystemBase {

    public enum IntakePosition {
        SourcePickup(IntakeConstants.POSITIONS.SOURCE_PICKUP),
        GroundPickup(IntakeConstants.POSITIONS.GROUND_PICKUP),
        Stowed(IntakeConstants.POSITIONS.STOWED),
        AmpScore(IntakeConstants.POSITIONS.AMP_SCORE),
        SpeakerScore(IntakeConstants.POSITIONS.SPEAKER_SCORE);

        private final LoggedTunableNumber _angle;

        /**
         * These enum represents a prefedined position of the intake. These are what we pass to
         * commands to set the intake positions
         * @param angle
         */
        private IntakePosition(LoggedTunableNumber angle) {
            this._angle = angle;
        }

        /**
         * Return the angle associated with a given position. These angles are defined in IntakeConstants.java
         * @return
         */
        public double getAngle() {
            return this._angle.get();
        }
    }

    protected IntakePosition _intakePosition;
    private final IntakeCommandFactory _intakeCommandFactory = new IntakeCommandFactory(this);

    private final IntakeIO _intakeIO;
    private final IntakeIOInputsAutoLogged _intakeInputs = new IntakeIOInputsAutoLogged();

    private boolean _hasNote;
    private boolean _intakeHasBeenRunning;
    private ProfiledPIDController _pivotPid;
    private IntakeVisualizer _intakeVisualizer = new IntakeVisualizer();

    public Intake(IntakeIO intakeIO) {
        // TODO: Make sure this is updated in the periodic().
        this._intakeIO = intakeIO;
        _hasNote = false;
        _intakeHasBeenRunning = false;
        _intakePosition = IntakePosition.Stowed;
        _pivotPid = new ProfiledPIDController(IntakeConstants.PID.PIVOT.KP.get(), 
                                            IntakeConstants.PID.PIVOT.KI.get(), 
                                            IntakeConstants.PID.PIVOT.KD.get(), 
                                            new Constraints(
                                                IntakeConstants.PID.PIVOT.MAXIMUM_VELOCITY.get(),
                                                IntakeConstants.PID.PIVOT.MAXIMUM_ACCELERATION.get()));

        _pivotPid.setIntegratorRange(-IntakeConstants.PID.PIVOT.MAXIMUM_INTEGRAL_SUM.get(), IntakeConstants.PID.PIVOT.MAXIMUM_INTEGRAL_SUM.get());                                        
        // might want to call this to make sure inputs are always initialized??
        // this.periodic();
        this.setIntakePosition(IntakePosition.Stowed);
    }

    /**
     * Sets the intake motor speed using the intakeIO interface. 
     * Sets the intake motor speed to the result of the PID controller.
     * 
     */
    public void runPivotPID() {
        _intakeIO.setPivotMotorSpeed(_pivotPid.calculate(getPivotAngle()));
    }

    /**
     * returns the position of the intake but not in degrees. 
     * Returns one of the known positions such as stowed, ground pickup, 
     * source, amp, etc
     * @return intakePosition
     */
    public IntakePosition getIntakePosition() {
        return this._intakePosition;
    }

    /**
     * Set intake angle based on a passed in position such as
     * amp, ground pickup, stowed, etc
     * @param position
     */
    public void setIntakePosition(IntakePosition position) {
        this._intakePosition = position;
        setIntakePositionWithAngle(position.getAngle());
    }

    /**
     * Sets the roller motor to the acquire speed
     */
    public void setRollerMotorSpeedAcquire() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.ROLLERS.AQUIRE_SPEED.get());
    }

    /**
     * Sets the roller motor to the spit speed
     */
    public void setRollerMotorSpeedSpit() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.ROLLERS.SPIT_SPEED.get());
    }

    /**
     * Stops the roller motor
     */
    public void stopRollerMotor() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.ROLLERS.STOP_SPEED.get());
    }

    /**
     * Sets the intake position based on the angle. If the angle is too small or big, return nothing.
     * @param angle
     */
    public void setIntakePositionWithAngle(Double angle) {
        if (angle > IntakeConstants.MECHANICAL.MAX_ANGLE || angle < IntakeConstants.MECHANICAL.MIN_ANGLE) {
            // TODO: log an error, but don't throw exception
            return;
        }
        //_intakeIO.setTargetPositionAsDegrees(angle);
        _pivotPid.reset(getPivotAngle());
        _pivotPid.setGoal(angle);
    }

    /**
     * Checks if the pivot is at the desired setpoint using encoder position.
     * @return boolean
     */
    public boolean pivotAtSetpoint() {
        double pivotEncoderPositionDegrees = _intakeInputs._pivotEncoderPositionDegrees;
        double targetPositionDegrees = _intakePosition.getAngle();
        System.out.printf("angle offset: %f %f\n", pivotEncoderPositionDegrees, targetPositionDegrees);
        return Math.abs(pivotEncoderPositionDegrees - targetPositionDegrees) <= IntakeConstants.SOFTWARE.PIVOT_DEADBAND;
    }

    /**
     * Checks if the intake has a note
     * @return _hasNote
     */
    public boolean hasNote() {
        if (!_hasNote) {
            _hasNote = (_intakeInputs._rollerMotorCurrent > IntakeConstants.SOFTWARE.ROLLER_CURRENT_LIMIT) && _intakeHasBeenRunning;
        }

        return _hasNote;
    }

    /**
     * Sets _hasNote to false
     */
    public void resetHasNote() {
        _hasNote = false;
    }

    /**
     * Set has note to true. Needed as a work around for autos for the time being.
     * 
     * Using this to tell the robot it has a note so it can do auto shoot
     */
    public void setHasNote() {
        _hasNote = true;
    }

    /**
     * Sets intakeHasBeenRunning to a boolean
     * @param running
     */
    public void setIntakeHasBeenRunning(boolean running) {
        _intakeHasBeenRunning = running;
    }

    /**
     * Returns the pivot angle
     * @return pivotEncoderPositionDegrees
     */
    public double getPivotAngle() {
        return _intakeInputs._pivotEncoderPositionDegrees;
    }

    /**
     * Returns the target position
     * @return intakePosition as an angle (double)
     */
    public double getTargetPosition(){
        return this._intakePosition.getAngle();
    }

    /**
     * this is how we call commands to run the intake. to call a command use 
     * "intake.buildCommand.COMMANDTORUN(COMMAND ARGUMENTS)". replace command
     * to run with a command from {@link IntakeCommandFactory} and replace 
     * COMMAND ARGUMENTS with the arguments needed for that command, which could be empty.
     * @return
     */
    public IntakeCommandFactory buildCommand() {
        return this._intakeCommandFactory;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _intakeIO.updateInputs(_intakeInputs);
        Logger.processInputs("Intake", _intakeInputs);

        //smart dashboard testing
        SmartDashboard.putNumber("Intake PID Speed", _pivotPid.calculate(_intakeInputs._pivotEncoderPositionDegrees));
        SmartDashboard.putNumber("Intake Current Pivot Angle", _intakeInputs._pivotEncoderPositionDegrees);
        SmartDashboard.putNumber("Intake Desired Pivot Angle", _pivotPid.getSetpoint().position);
        SmartDashboard.putNumber("Intake PID Error", _pivotPid.getPositionError());
        SmartDashboard.putBoolean("Intake Has Note", _hasNote);

        // VISUALIZATION
        double angle = _intakeInputs._pivotEncoderPositionDegrees;
        _intakeVisualizer.update(angle);

        // TUNABLES FOR PIVOT PID

        if (IntakeConstants.PID.PIVOT.KP.hasChanged(hashCode())) {
            _pivotPid.setP(IntakeConstants.PID.PIVOT.KP.get());
        }

        if (IntakeConstants.PID.PIVOT.KI.hasChanged(hashCode())) {
            _pivotPid.setI(IntakeConstants.PID.PIVOT.KI.get());
        }

        if (IntakeConstants.PID.PIVOT.KD.hasChanged(hashCode())) {
            _pivotPid.setD(IntakeConstants.PID.PIVOT.KD.get());
        }

        if (IntakeConstants.PID.PIVOT.MAXIMUM_VELOCITY.hasChanged(hashCode())) {
            _pivotPid.setConstraints(new Constraints(
                IntakeConstants.PID.PIVOT.MAXIMUM_VELOCITY.get(),
                IntakeConstants.PID.PIVOT.MAXIMUM_ACCELERATION.get()));
        }

        if (IntakeConstants.PID.PIVOT.MAXIMUM_ACCELERATION.hasChanged(hashCode())) {
            _pivotPid.setConstraints(new Constraints(
                IntakeConstants.PID.PIVOT.MAXIMUM_VELOCITY.get(),
                IntakeConstants.PID.PIVOT.MAXIMUM_ACCELERATION.get()));
        }

        if (IntakeConstants.PID.PIVOT.MAXIMUM_INTEGRAL_SUM.hasChanged(hashCode())) {
            _pivotPid.setIntegratorRange(-IntakeConstants.PID.PIVOT.MAXIMUM_INTEGRAL_SUM.get(), IntakeConstants.PID.PIVOT.MAXIMUM_INTEGRAL_SUM.get());
        }

        // TUNABLES FOR ROLLERS

        if (IntakeConstants.ROLLERS.AQUIRE_SPEED.hasChanged(hashCode())) {
            _intakeIO.setRollerMotorSpeed(IntakeConstants.ROLLERS.AQUIRE_SPEED.get());
        }

        if (IntakeConstants.ROLLERS.SPIT_SPEED.hasChanged(hashCode())) {
            _intakeIO.setRollerMotorSpeed(IntakeConstants.ROLLERS.SPIT_SPEED.get());
        }

        if (IntakeConstants.ROLLERS.SPIT_TIME.hasChanged(hashCode())) {
            _intakeIO.setRollerMotorSpeed(IntakeConstants.ROLLERS.SPIT_SPEED.get());
        }

        if (IntakeConstants.ROLLERS.CURRENT_CUTOFF.hasChanged(hashCode())) {
            _intakeIO.setRollerMotorSpeed(IntakeConstants.ROLLERS.STOP_SPEED.get());
        }

        if (IntakeConstants.ROLLERS.STOP_SPEED.hasChanged(hashCode())) {
            _intakeIO.setRollerMotorSpeed(IntakeConstants.ROLLERS.STOP_SPEED.get());
        }



        // LOGGING
        Logger.recordOutput("Intake/PID Speed", _pivotPid.calculate(_intakeInputs._pivotEncoderPositionDegrees));
        Logger.recordOutput("Intake/Current-Pivot-Angle", _intakeInputs._pivotEncoderPositionDegrees);
        Logger.recordOutput("Intake/Desired-Pivot-Angle", _pivotPid.getSetpoint().position);
        Logger.recordOutput("Intake/PID-Error", _pivotPid.getPositionError());
        Logger.recordOutput("Intake/Has-Note", _hasNote);
    }
}

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged; checkstyle says this is redunant

public class Intake extends SubsystemBase {
    public enum IntakePosition {
        SourcePickup(IntakeConstants.POSITION_SOURCE_PICKUP),
        GroundPickup(IntakeConstants.POSITION_GROUND_PICKUP),
        Stowed(IntakeConstants.POSITION_STOWED), //5.0
        AmpScore(IntakeConstants.POSITION_AMP_SCORE),
        SpeakerScore(IntakeConstants.POSITION_SPEAKER_SCORE);

        private final double _angle;

        private IntakePosition(double angle) {
            this._angle = angle;
        }

        public double getAngle() {
            return this._angle;
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
        this._intakeIO = intakeIO;
        _hasNote = false;
        _intakeHasBeenRunning = false;
        _intakePosition = IntakePosition.Stowed;
        _pivotPid = new ProfiledPIDController(IntakeConstants.PIVOT_PID_KP, 
                                            IntakeConstants.PIVOT_PID_KI, 
                                            IntakeConstants.PIVOT_PID_KD, 
                                            new Constraints(
                                                IntakeConstants.PIVOT_PID_MAX_VEL,
                                                IntakeConstants.PIVOT_PID_MAX_ACCEL));

        _pivotPid.setIntegratorRange(-IntakeConstants.PIVOT_PID_MAX_ISUM, IntakeConstants.PIVOT_PID_MAX_ISUM);                                        
        // might want to call this to make sure inputs are always initialized??
        // this.periodic();
        this.setIntakePosition(IntakePosition.Stowed);

        // REMOVE THESE COMMENTS IF IT DOESN'T WORK :)
        // setDefaultCommand(_intakeCommandFactory.runPID());
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
        _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE_ACQUIRE_SPEED);
    }

    /**
     * Sets the roller motor to the spit speed
     */
    public void setRollerMotorSpeedSpit() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE_SPIT_SPEED);
    }

    /**
     * Stops the roller motor
     */
    public void stopRollerMotor() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE_STOP_SPEED);
    }

    /**
     * Sets the intake position based on the angle. If the angle is too small or big, return nothing.
     * @param angle
     */
    public void setIntakePositionWithAngle(Double angle) {
        if (angle > IntakeConstants.MAX_INTAKE_ANGLE || angle < IntakeConstants.MIN_INTAKE_ANGLE) {
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
        return Math.abs(pivotEncoderPositionDegrees - targetPositionDegrees) <= IntakeConstants.INTAKE_PIVOT_DEADBAND;
    }

    /**
     * Checks if the intake has a note
     * @return _hasNote
     */
    public boolean hasNote() {
        if (!_hasNote) {
            _hasNote = (_intakeInputs._rollerMotorCurrent > IntakeConstants.INTAKE_CURRENT_CUTOFF) && _intakeHasBeenRunning;
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

        // LOGGING
        Logger.recordOutput("Intake/PID Speed", _pivotPid.calculate(_intakeInputs._pivotEncoderPositionDegrees));
        Logger.recordOutput("Intake/Current-Pivot-Angle", _intakeInputs._pivotEncoderPositionDegrees);
        Logger.recordOutput("Intake/Desired-Pivot-Angle", _pivotPid.getSetpoint().position);
        Logger.recordOutput("Intake/PID-Error", _pivotPid.getPositionError());
        Logger.recordOutput("Intake/Has-Note", _hasNote);
    }
}

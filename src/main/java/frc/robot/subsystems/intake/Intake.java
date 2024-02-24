package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.tunable.LoggedTunableNumber;
import frc.robot.util.tunable.Tunable;

public class Intake extends SubsystemBase implements Tunable{

    /** The position of the intake in the various positions that it could be in during a cycle. */
    public enum IntakePosition {
        SourcePickup(
            IntakeConstants.POSITIONS.POSITION_SOURCE_PICKUP_DEGREES),
        GroundPickup(
            IntakeConstants.POSITIONS.POSITION_GROUND_PICKUP_DEGREES),
        Stowed(
            IntakeConstants.POSITIONS.POSITION_STOWED_DEGREES), 
        AmpScore(
            IntakeConstants.POSITIONS.POSITION_AMP_SCORE_DEGREES),
        SpeakerScore(
            IntakeConstants.POSITIONS.POSITION_SPEAKER_SCORE_DEGREES);

        // The LoggedTunableNumber is a wrapper around the double that is the angle.
        private LoggedTunableNumber _angle;

        private IntakePosition(LoggedTunableNumber angle) {
            this._angle = angle;
        }

        public double getAngle() {
            return _angle.get();
        }
    }

    protected IntakePosition _intakePosition;
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

        // It should be noted that we must in our poll() method check and update this PID. 
        _pivotPid = new ProfiledPIDController(IntakeConstants.PID.PIVOT.KP.get(), 
                                                IntakeConstants.PID.PIVOT.KI.get(), 
                                                IntakeConstants.PID.PIVOT.KD.get(), 
                                            new Constraints(
                                                IntakeConstants.PID.PIVOT.PIVOT_PID_MAX_VELOCITY_METERS_SECOND.get(),
                                                IntakeConstants.PID.PIVOT.PIVOT_PID_MAX_ACCCELERATION_METERS_SECOND_SQUARE.get()));
    }

    public void runPivotPID() {
        _intakeIO.setPivotMotorSpeed(_pivotPid.calculate(getPivotAngle()));
    }

    public IntakePosition getIntakePosition() {
        return this._intakePosition;
    }

    public void setIntakePosition(IntakePosition position) {
        this._intakePosition = position;
        setIntakePositionWithAngle(this._intakePosition.getAngle());
    }

    /** Set the motors on the motor to a specified speed to intake a piece. */
    public void setRollerMotorSpeedAcquire() {
        if (IntakeConstants.INTAKE.INTAKE_ACQUIRE_SPEED.get() > 0) {
            _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE.INTAKE_ACQUIRE_SPEED.get());
            return;
        }
        _intakeIO.setRollerMotorSpeed(0.0); // If we have an error it's best to just do nothing and print it.
        System.err.println("[ERROR]: Intake acquire speed is negative.");
    }

    /** Set the motors on the motor to a specified speed to spit out a piece. */
    public void setRollerMotorSpeedSpit() {
        if (IntakeConstants.INTAKE.INTAKE_SPIT_SPEED.get() < 0) {
            _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE.INTAKE_SPIT_SPEED.get());
            return;
        }
        _intakeIO.setRollerMotorSpeed(0.0);
        System.err.println("[ERROR]: Intake spit speed is positive.");
    }

    /** Set the motors on the motor to a specified idle speed. */
    public void stopRollerMotor() {
        // Since we don't know whether the idle speed will be positive or negative there is no check here.
        _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE.INTAKE_IDLE_SPEED.get());
    }

    public void setIntakePositionWithAngle(Double angle) {
        if (angle > IntakeConstants.MECHANICAL.MAX_INTAKE_ANGLE_DEGREES || angle < IntakeConstants.MECHANICAL.MIN_INTAKE_ANGLE_DEGREES) {
            System.out.println("[ERROR]: Intake angle out of bounds");
            return;
        }
        _pivotPid.reset(getPivotAngle());
        _pivotPid.setGoal(angle);
    }

    public boolean pivotAtSetpoint() {
        double pivotEncoderPositionDegrees = Units.rotationsToDegrees(_intakeInputs._pivotEncoderPositionDegrees);
        double targetPositionDegrees = _intakePosition.getAngle();
        return Math.abs(pivotEncoderPositionDegrees - targetPositionDegrees) <= IntakeConstants.SOFTWARE.INTAKE_PIVOT_DEADBAND_DEGREES.get();
    }

    public boolean hasNote() {
        if (!_hasNote) {
            _hasNote = (_intakeInputs._rollerMotorCurrent > IntakeConstants.INTAKE.INTAKE_CURRENT_CUTOFF.get()) && _intakeHasBeenRunning;
        }

        return _hasNote;
    }

    public void resetHasNote() {
        _hasNote = false;
    }

    public void setIntakeHasBeenRunning(boolean running) {
        _intakeHasBeenRunning = running;
    }

    public double getPivotAngle() {
        return this._intakeInputs._pivotEncoderPositionDegrees;
    }

    public double getTargetPosition(){
        return this._intakePosition.getAngle();
    }

    @Override
    public void periodic() {
        _intakeIO.updateInputs(_intakeInputs);
        Logger.processInputs("Intake", _intakeInputs);

        // SMART DASHBOARD TESTING
        SmartDashboard.putNumber("Intake PID Speed", _pivotPid.calculate(_intakeInputs._pivotEncoderPositionDegrees));
        SmartDashboard.putNumber("Intake Current Pivot Angle", _intakeInputs._pivotEncoderPositionDegrees);
        SmartDashboard.putNumber("Intake Desired Pivot Angle", _pivotPid.getSetpoint().position);
        SmartDashboard.putNumber("Intake PID Error", _pivotPid.getPositionError());
        SmartDashboard.putBoolean("Intake Has Note", _hasNote);

        // VISUALIZATION
        double angle = _intakeInputs._pivotEncoderPositionDegrees;
        _intakeVisualizer.update(angle);

        // LOGGING
        Logger.recordOutput("Intake/AngleDegrees", angle);
        Logger.recordOutput("Intake/PID Speed", _pivotPid.calculate(_intakeInputs._pivotEncoderPositionDegrees));
        Logger.recordOutput("Intake/Current-Pivot-Angle", _intakeInputs._pivotEncoderPositionDegrees);
        Logger.recordOutput("Intake/Desired-Pivot-Angle", _pivotPid.getSetpoint().position);
        Logger.recordOutput("Intake/PID-Error", _pivotPid.getPositionError());
        Logger.recordOutput("Intake/Has-Note", _hasNote);

        // TUNABLES
        this.poll();
    }

    @Override
    public void poll() {
        _intakeIO.poll();

        if (IntakeConstants.PID.PIVOT.KP.hasChanged(hashCode())){
            _pivotPid.setP(IntakeConstants.PID.PIVOT.KP.get());
        }

        if (IntakeConstants.PID.PIVOT.KI.hasChanged(hashCode())){
            _pivotPid.setI(IntakeConstants.PID.PIVOT.KI.get());
        }

        if (IntakeConstants.PID.PIVOT.KD.hasChanged(hashCode())){
            _pivotPid.setD(IntakeConstants.PID.PIVOT.KD.get());
        }

        if (IntakeConstants.PID.PIVOT.KF.hasChanged(hashCode())){
            // This doesn't do anything right now but in case we want a feedfoward its here.
        }

        if (IntakeConstants.PID.PIVOT.PIVOT_PID_MAX_VELOCITY_METERS_SECOND.hasChanged(hashCode())){
            _pivotPid.setConstraints(new Constraints(
                IntakeConstants.PID.PIVOT.PIVOT_PID_MAX_VELOCITY_METERS_SECOND.get(),
                IntakeConstants.PID.PIVOT.PIVOT_PID_MAX_ACCCELERATION_METERS_SECOND_SQUARE.get()));
        }

        if (IntakeConstants.PID.PIVOT.PIVOT_PID_MAX_ACCCELERATION_METERS_SECOND_SQUARE.hasChanged(hashCode())){
            _pivotPid.setConstraints(new Constraints(
                IntakeConstants.PID.PIVOT.PIVOT_PID_MAX_VELOCITY_METERS_SECOND.get(),
                IntakeConstants.PID.PIVOT.PIVOT_PID_MAX_ACCCELERATION_METERS_SECOND_SQUARE.get()));
        }
    }
}

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {

    public enum IntakePosition {
        SourcePickup(IntakeConstants.POSITIONS.POSITION_SOURCE_PICKUP.get()), // TODO: There is probably a better way to do this (since we're polling each cycle) but it'll work for now. -MG
        GroundPickup(IntakeConstants.POSITIONS.POSITION_GROUND_PICKUP.get()),
        Stowed(IntakeConstants.POSITIONS.POSITION_STOWED.get()), //5.0
        AmpScore(IntakeConstants.POSITIONS.POSITION_AMP_SCORE.get()),
        SpeakerScore(IntakeConstants.POSITIONS.POSITION_SPEAKER_SCORE.get());

        private final double _angle;

        private IntakePosition(double angle) {
            this._angle = angle;
        }

        public double getAngle() {
            return this._angle;
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
        _pivotPid = new ProfiledPIDController(IntakeConstants.PID.PIVOT.KP, 
                                            IntakeConstants.PID.PIVOT.KI, 
                                            IntakeConstants.PID.PIVOT.KD, 
                                            new Constraints(
                                                IntakeConstants.PID.PIVOT.PIVOT_PID_MAX_VEL,
                                                IntakeConstants.PID.PIVOT.PIVOT_PID_MAX_ACCEL));

        // might want to call this to make sure inputs are always initialized??
        // this.periodic();
    }

    public void runPivotPID() {
        _intakeIO.setPivotMotorSpeed(_pivotPid.calculate(getPivotAngle()));
    }

    public IntakePosition getIntakePosition() {
        return this._intakePosition;
    }

    public void setIntakePosition(IntakePosition position) {
        this._intakePosition = position;
        setIntakePositionWithAngle(position.getAngle());
    }

    public void setRollerMotorSpeedAcquire() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE.INTAKE_ACQUIRE_SPEED_DEFAULT);
    }

    public void setRollerMotorSpeedSpit() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE.INTAKE_SPIT_SPEED_DEFAULT);
    }

    public void stopRollerMotor() {
        _intakeIO.setRollerMotorSpeed(0.02);
    }

    public void setIntakePositionWithAngle(Double angle) {
        if (angle > IntakeConstants.MECHANICAL.MAX_INTAKE_ANGLE || angle < IntakeConstants.MECHANICAL.MIN_INTAKE_ANGLE) {
            System.out.println("[ERROR]: Intake angle out of bounds");
            return;
        }
        //_intakeIO.setTargetPositionAsDegrees(angle);
        _pivotPid.reset(getPivotAngle());
        _pivotPid.setGoal(angle);
    }

    public boolean pivotAtSetpoint() {
        double pivotEncoderPositionDegrees = Units.rotationsToDegrees(_intakeInputs._pivotEncoderPositionDegrees);
        double targetPositionDegrees = _intakePosition.getAngle();
        return Math.abs(pivotEncoderPositionDegrees - targetPositionDegrees) <= IntakeConstants.SOFTWARE.INTAKE_PIVOT_DEADBAND;
    }

    public boolean hasNote() {
        if (!_hasNote) {
            _hasNote = (_intakeInputs._rollerMotorCurrent > IntakeConstants.INTAKE.INTAKE_CURRENT_CUTOFF_DEFAULT) && _intakeHasBeenRunning;
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
        return _intakeInputs._pivotEncoderPositionDegrees;
    }

    public double getTargetPosition(){
        return this._intakePosition.getAngle();
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
        Logger.recordOutput("Intake/AngleDegrees", angle);
        Logger.recordOutput("Intake/PID Speed", _pivotPid.calculate(_intakeInputs._pivotEncoderPositionDegrees));
        Logger.recordOutput("Intake/Current-Pivot-Angle", _intakeInputs._pivotEncoderPositionDegrees);
        Logger.recordOutput("Intake/Desired-Pivot-Angle", _pivotPid.getSetpoint().position);
        Logger.recordOutput("Intake/PID-Error", _pivotPid.getPositionError());
        Logger.recordOutput("Intake/Has-Note", _hasNote);
    }

    private void pollForTunableChanges() {
        if (IntakeConstants.PID.PIVOT.KP_TUNABLE.hasChanged(hashCode())){
            IntakeConstants.PID.PIVOT.KP = IntakeConstants.PID.PIVOT.KP_TUNABLE.get();
            _pivotPid.setP(IntakeConstants.PID.PIVOT.KP);
        }

        if (IntakeConstants.PID.PIVOT.KI_TUNABLE.hasChanged(hashCode())){
            IntakeConstants.PID.PIVOT.KI = IntakeConstants.PID.PIVOT.KI_TUNABLE.get();
            _pivotPid.setI(IntakeConstants.PID.PIVOT.KI);
        }

        if (IntakeConstants.PID.PIVOT.KD_TUNABLE.hasChanged(hashCode())){
            IntakeConstants.PID.PIVOT.KD = IntakeConstants.PID.PIVOT.KD_TUNABLE.get();
            _pivotPid.setD(IntakeConstants.PID.PIVOT.KD);
        }

        if (IntakeConstants.PID.PIVOT.KF_TUNABLE.hasChanged(hashCode())){
            IntakeConstants.PID.PIVOT.KF = IntakeConstants.PID.PIVOT.KF_TUNABLE.get();
            // This doesn't do anything right now but in case we want a feedfoward its here.
        }


    }
}

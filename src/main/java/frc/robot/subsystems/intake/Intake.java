package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.intake.IntakeIO;
import frc.robot.hardware.intake.IntakeIOInputsAutoLogged;

public class Intake extends SubsystemBase {
    public enum IntakePosition {
        SourcePickup(75),
        GroundPickup(10),
        Stowed(100),
        AmpScore(80),
        SpeakerScore(115);

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
    private PIDController _pivotPid;

    public Intake(IntakeIO intakeIO) {
        this._intakeIO = intakeIO;
        _hasNote = false;
        _pivotPid = new PIDController(0, 0, 0);
    }

    public void runPivotPID() {
        //_intakeIO.setPivotMotorSpeed(_pivotPid.calculate(getPivotMotorAngle()));
    }

    public IntakePosition getIntakePosition() {
        return this._intakePosition;
    }

    public void setIntakePosition(IntakePosition position) {
        _intakePosition = position;
        setIntakePositionWithAngle(position.getAngle());
    }

    public void setRollerMotorSpeedAcquire() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE_ACQUIRE_SPEED);
    }

    public void setRollerMotorSpeedSpit() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE_SPIT_SPEED);
    }

    public void stopRollerMotor() {
        _intakeIO.setRollerMotorSpeed(0);
    }

    public void setIntakePositionWithAngle(Double angle) {
        if (angle > IntakeConstants.MAX_INTAKE_ANGLE || angle < IntakeConstants.MIN_INTAKE_ANGLE) {
            // TODO: log an error, but don't throw exception
            return;
        }
        //_intakeIO.setTargetPositionAsDegrees(angle);
        _pivotPid.setSetpoint(angle);
    }

    public boolean pivotAtSetpoint() {
        double pivotEncoderPositionDegrees = Units.rotationsToDegrees(_intakeInputs._pivotEncoderPositionDegrees);
        double targetPositionDegrees = _intakePosition.getAngle();
        return Math.abs(pivotEncoderPositionDegrees - targetPositionDegrees) <= IntakeConstants.INTAKE_PIVOT_DEADBAND;
    }

    public boolean hasNote() {
        if (!_hasNote) {
            _hasNote = _intakeInputs._rollerMotorCurrent > IntakeConstants.INTAKE_CURRENT_CUTOFF;
        }

        return _hasNote;
    }

    public void resetHasNote() {
        _hasNote = false;
    }

    public double getPivotMotorAngle() {
        return _intakeInputs._pivotMotorPositionDegrees;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _intakeIO.updateInputs(_intakeInputs);
        Logger.processInputs("Intake", _intakeInputs);
        SmartDashboard.putNumber("Intake PID Speed", _pivotPid.calculate(getPivotMotorAngle()));
        SmartDashboard.putNumber("Intake Current Pivot Angle", getPivotMotorAngle());
        SmartDashboard.putNumber("Intake Desired Pivot Angle", _pivotPid.getSetpoint());

    }
}

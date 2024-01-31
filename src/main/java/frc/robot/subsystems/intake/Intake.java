package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.intake.IntakeIO;
import frc.robot.hardware.intake.IntakeIOInputsAutoLogged;

public class Intake extends SubsystemBase {
    public enum IntakePosition {
        SourcePickup (75),
        GroundPickup (10),
        Stowed (100),
        AmpScore (80),
        SpeakerScore (115);

        private final double _angle;

        private IntakePosition(double angle) {
            this._angle = angle;
        }

        public double get_angle() {
            return this._angle;
        }
    }

    protected IntakePosition _intakePosition;
    private final IntakeIO _intakeIO;
    private final IntakeIOInputsAutoLogged _intakeInputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO intakeIO) {
        this._intakeIO = intakeIO;
    }

    public IntakePosition getIntakePosition() {
        return this._intakePosition;
    }

    public void setIntakePosition(IntakePosition position) {
        _intakePosition = position;
        setIntakePositionWithAngle(position.get_angle());
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
        _intakeIO.setTargetPositionAsDegrees(angle);
    }

    public boolean pivotAtSetpoint() {
        double pivotEncoderPositionDegrees = Units.rotationsToDegrees(_intakeInputs._pivotEncoderPositionRotations);
        double targetPositionDegrees = _intakePosition.get_angle();
        return Math.abs(pivotEncoderPositionDegrees - targetPositionDegrees) <= IntakeConstants.INTAKE_PIVOT_DEADBAND;
    }

    public boolean hasNote() {
        return _intakeInputs._rollerMotorCurrent > IntakeConstants.INTAKE_CURRENT_CUTOFF;
    }

    public double getPivotMotorAngle() {
        return _intakeInputs._pivotMotorPositionDegrees;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _intakeIO.updateInputs(_intakeInputs);
        Logger.processInputs("Intake", _intakeInputs);
    }
}

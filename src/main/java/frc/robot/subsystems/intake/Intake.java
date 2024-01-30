package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.intake.IntakeIO;
import frc.robot.hardware.intake.IntakeIOInputsAutoLogged;

public class Intake extends SubsystemBase {
    public enum IntakePosition {
        SourcePickup,
        GroundPickup,
        Stowed,
        AmpScore,
        SpeakerScore
    }

    protected IntakePosition _intakePosition;
    private final IntakeIO _intakeIO;
    private final IntakeIOInputsAutoLogged _intakeInputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO intakeIO) {
        this._intakeIO = intakeIO;
    }

    @AutoLogOutput
    public IntakePosition getIntakePosition() {
        return this._intakePosition;
    }

    public void setIntakePosition(IntakePosition position) {
         _intakePosition = position;
         switch (_intakePosition) {
             case GroundPickup:
                 setIntakePositionWithAngle(IntakeConstants.INTAKE_GROUND_PICKUP_ANGLE);
                 break;
             case SourcePickup:
                 setIntakePositionWithAngle(IntakeConstants.INTAKE_SOURCE_PICKUP_ANGLE);
                 break;
             case Stowed:
                 setIntakePositionWithAngle(IntakeConstants.INTAKE_STOWED_ANGLE);
                 break;
             case AmpScore:
                 setIntakePositionWithAngle(IntakeConstants.INTAKE_AMP_SCORE_ANGLE);
                 break;
             case SpeakerScore:
                 setIntakePositionWithAngle(IntakeConstants.INTAKE_SPEAKER_SCORE_ANGLE);
                 break;
         }
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

    @AutoLogOutput
    public boolean pivotAtSetpoint() {
        double pivotEncoderPositionDegrees = Units.rotationsToDegrees(_intakeInputs._pivotEncoderPositionRotations);
        double targetPositionDegrees = 0;
        switch (_intakePosition) {
             case GroundPickup:
                 targetPositionDegrees = IntakeConstants.INTAKE_GROUND_PICKUP_ANGLE;
                 break;
             case SourcePickup:
                 targetPositionDegrees = IntakeConstants.INTAKE_SOURCE_PICKUP_ANGLE;
                 break;
             case Stowed:
                 targetPositionDegrees = IntakeConstants.INTAKE_STOWED_ANGLE;
                 break;
             case AmpScore:
                 targetPositionDegrees = IntakeConstants.INTAKE_AMP_SCORE_ANGLE;
                 break;
             case SpeakerScore:
                 targetPositionDegrees = IntakeConstants.INTAKE_SPEAKER_SCORE_ANGLE;
                 break;
         }
        return pivotEncoderPositionDegrees == targetPositionDegrees;
    }

    @AutoLogOutput
    public boolean hasNote() {
        return _intakeInputs._rollerMotorCurrent > IntakeConstants.INTAKE_CURRENT_CUTOFF;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _intakeIO.updateInputs(_intakeInputs);
        Logger.processInputs("Intake", _intakeInputs);
    }
}

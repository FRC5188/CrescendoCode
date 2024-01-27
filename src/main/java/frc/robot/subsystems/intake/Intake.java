package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.hardware.intake.IntakeHardware;

public class Intake extends SubsystemBase {
    public enum IntakePosition {
        SourcePickup,
        GroundPickup,
        Stowed,
        AmpScore,
        SpeakerScore
    }

    protected IntakePosition _intakePosition;

    //private IntakeHardware _hardware;

    //public Intake(IntakeHardware hardware) {
    //    _hardware = hardware;
    //}

    public IntakePosition getIntakePosition() {
        //return this._intakePosition;
        return null;
    }

    // public void setIntakePosition(IntakePosition position) {
    //     _intakePosition = position;
    //     switch (_intakePosition) {
    //         case GroundPickup:
    //             setIntakePositionWithAngle(IntakeConstants.INTAKE_GROUND_PICKUP_ANGLE);
    //             break;
    //         case SourcePickup:
    //             setIntakePositionWithAngle(IntakeConstants.INTAKE_SOURCE_PICKUP_ANGLE);
    //             break;
    //         case Stowed:
    //             setIntakePositionWithAngle(IntakeConstants.INTAKE_STOWED_ANGLE);
    //             break;
    //         case AmpScore:
    //             setIntakePositionWithAngle(IntakeConstants.INTAKE_AMP_SCORE_ANGLE);
    //             break;
    //         case SpeakerScore:
    //             setIntakePositionWithAngle(IntakeConstants.INTAKE_SPEAKER_SCORE_ANGLE);
    //             break;
    //     }
    // }

    //public void setRollerMotorSpeedAcquire() {
    //    _hardware.getRollerMotor().set(IntakeConstants.INTAKE_ACQUIRE_SPEED);
    //}

    //public void setRollerSpeedSpit() {
    //    _hardware.getRollerMotor().set(IntakeConstants.INTAKE_SPIT_SPEED);
    //}

    //public void stopRollerMotor() {
    //    _hardware.getRollerMotor().set(0);
    //}

    //public void setIntakePositionWithAngle(Double angle) {
    //    if (angle > IntakeConstants.MAX_INTAKE_ANGLE || angle < IntakeConstants.MIN_INTAKE_ANGLE) {
    //        // TODO: log an error, but don't throw exception
    //        return;
    //    } 
    //    _hardware.getPivotMotorPID().setSetpoint(angle);
    //}

    //public boolean pivotAtSetpoint() {
    //    return _hardware.getPivotMotorPID().atSetpoint();
    //}

    //public boolean hasNote() {
    //    return _hardware.getRollerMotor().getOutputCurrent() > IntakeConstants.INTAKE_CURRENT_CUTOFF;
    //}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

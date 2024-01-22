package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.intake.IntakeHardware;

public class Intake extends SubsystemBase {
    public enum IntakePosition {
        SourcePickup,
        GroundPickup,
        Stowed,
        AmpScore,
        SpeakerScore
    }

    protected IntakePosition _intakePosition;

    private IntakeHardware _hardware;

    public Intake(IntakeHardware hardware) {
        _hardware = hardware;
    }

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

    public void setIntakePositionWithAngle(Double angle) {
        // TODO: Implement
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

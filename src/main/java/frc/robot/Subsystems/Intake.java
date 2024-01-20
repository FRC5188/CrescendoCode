package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public enum IntakePosition {
        SourcePickup, GroundPickup, Stowed, AmpScore
    }
    protected final double INTAKE_GROUND_PICKUP = 0; //subject to change. Hi programmer!
    protected final double INTAKE_SOURCE_PICKUP = 0; //also subject to change!
    protected final double INTAKE_STOWED = 0; //subject to change. Hi programmer!
    protected final double INTAKE_AMP_SCORE = 0; //also subject to change!
    protected final double INTAKE_ACQUIRE_SPEED = 0; //subject to change
    protected final double INTAKE_SPIT_SPEED = 0; //also subject to change!
    protected IntakePosition _intakePosition;
    protected PIDController _pivotMotorPID;
    protected DutyCycleEncoder _pivotAbsEncoder;

    protected CANSparkFlex _rollerMotor;

    public IntakePosition getIntakePosition() {
            return this._intakePosition;
    }

    public void setIntakePosition(IntakePosition dontKillMeZoe) {
        _intakePosition = dontKillMeZoe;
        switch (_intakePosition) {
            case GroundPickup:
                _pivotMotorPID.setSetpoint(INTAKE_GROUND_PICKUP);
                break;
            case SourcePickup:
                _pivotMotorPID.setSetpoint(INTAKE_SOURCE_PICKUP);
                break;
            case Stowed:
                _pivotMotorPID.setSetpoint(INTAKE_STOWED);
                break;
            case AmpScore:
                _pivotMotorPID.setSetpoint(INTAKE_AMP_SCORE);
                break;
        }
    }

    public double getPivotMotorAngle() {
        return this._pivotAbsEncoder.getAbsolutePosition();
    }

    public void setRollerMotorSpeedAcquire() {
        _rollerMotor.set(INTAKE_ACQUIRE_SPEED);
    }

    public void setRollerSpeedSpit() {
        _rollerMotor.set(INTAKE_SPIT_SPEED);
    }

    public void stopRollerMotor() {
        _rollerMotor.set(0);
    }

    public boolean pivotAtSetpoint() {
        return _pivotMotorPID.atSetpoint();
    }
}

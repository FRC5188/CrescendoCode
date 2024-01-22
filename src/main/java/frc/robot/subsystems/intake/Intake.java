// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public enum IntakePosition {
        SourcePickup, GroundPickup, Stowed, AmpScore
    }

    protected final double INTAKE_GROUND_PICKUP = 0; // subject to change. Hi programmer!
    protected final double INTAKE_SOURCE_PICKUP = 0; // also subject to change!
    protected final double INTAKE_STOWED = 0; // subject to change. Hi programmer!
    protected final double INTAKE_AMP_SCORE = 0; // also subject to change!
    protected final double INTAKE_ACQUIRE_SPEED = 0; // subject to change
    protected final double INTAKE_SPIT_SPEED = 0; // also subject to change!
    protected final double PIVOT_MOTOR_KP = 0;
    protected final double PIVOT_MOTOR_KI = 0;
    protected final double PIVOT_MOTOR_KD = 0;
    protected final int PIVOT_ENCODER_CHANNEL = 0;
    protected final int ROLLER_MOTOR_ID = 0;

    protected IntakePosition _intakePosition;
    protected PIDController _pivotMotorPID;
    protected DutyCycleEncoder _pivotAbsEncoder;
    protected CANSparkFlex _rollerMotor;

    public Intake() {
        _intakePosition = IntakePosition.Stowed;
        _pivotMotorPID = new PIDController(PIVOT_MOTOR_KP, PIVOT_MOTOR_KI, PIVOT_MOTOR_KD);
        _pivotAbsEncoder = new DutyCycleEncoder(PIVOT_ENCODER_CHANNEL);
        _rollerMotor = new CANSparkFlex(ROLLER_MOTOR_ID, MotorType.kBrushless);
    }

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
        _rollerMotor.stopMotor();
    }

    public boolean pivotAtSetpoint() {
        return _pivotMotorPID.atSetpoint();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

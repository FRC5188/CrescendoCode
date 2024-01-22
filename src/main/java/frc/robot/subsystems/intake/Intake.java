// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.intake.IntakeHardware;

public class Intake extends SubsystemBase {
    // public enum IntakePosition {
    //     SourcePickup, 
    //     GroundPickup, 
    //     Stowed, 
    //     AmpScore
    // }

    protected final double INTAKE_GROUND_PICKUP = 0; // subject to change. Hi programmer!
    protected final double INTAKE_SOURCE_PICKUP = 0; // also subject to change!
    protected final double INTAKE_STOWED = 0; // subject to change. Hi programmer!
    protected final double INTAKE_AMP_SCORE = 0; // also subject to change!
    // protected IntakePosition _intakePosition;
    protected PIDController _pivotMotorPID;
    protected DutyCycleEncoder _pivotAbsEncoder;

    private IntakeHardware _hardware;

    public Intake(IntakeHardware hardware) {
        _hardware = hardware;
    }



    // public IntakePosition getIntakePosition() {
    //     return this._intakePosition;
    // }

    // public void setIntakePosition(IntakePosition dontKillMeZoe) {
    //     _intakePosition = dontKillMeZoe;
    //     switch (_intakePosition) {
    //         case GroundPickup:
    //             _pivotMotorPID.setSetpoint(INTAKE_GROUND_PICKUP);
    //             break;
    //         case SourcePickup:
    //             _pivotMotorPID.setSetpoint(INTAKE_SOURCE_PICKUP);
    //             break;
    //         case Stowed:
    //             _pivotMotorPID.setSetpoint(INTAKE_STOWED);
    //             break;
    //         case AmpScore:
    //             _pivotMotorPID.setSetpoint(INTAKE_AMP_SCORE);
    //             break;
    //     }
    // }

    public void setIntakePositionWithAngle(double angle) {
        
    }

    public double getPivotMotorAngle() {
        return this._pivotAbsEncoder.getAbsolutePosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

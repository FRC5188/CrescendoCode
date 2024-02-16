package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.intake.IntakeIO;
import frc.robot.hardware.intake.IntakeIOInputsAutoLogged;

public class Intake extends SubsystemBase {
    public enum IntakePosition {
        SourcePickup(50),
        GroundPickup(185),
        Stowed(5),
        AmpScore(60),
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
    private boolean _intakeHasBeenRunning;
    private ProfiledPIDController _pivotPid;
    private IntakeVisualizer _intakeVisualizer = new IntakeVisualizer();

    /**
     * Initializes Intake
     * @param intakeIO
     */
    public Intake(IntakeIO intakeIO) {
        this._intakeIO = intakeIO;
        _hasNote = false;
        _intakeHasBeenRunning = false;
        _pivotPid = new ProfiledPIDController(0.004, 0.0002, 0.001, new Constraints(50, 80));
    }

   // runs pivot motor PID once
    public void runPivotPID() {
        _intakeIO.setPivotMotorSpeed(_pivotPid.calculate(getPivotAngle()));
    }

    /**
     * returns curent intake position as enum
     * @return
     */
    public IntakePosition getIntakePosition() {
        return this._intakePosition;
    }

    /**
     * This method doesn’t move the intake, it only sets the PID’s setpoint based on the given position enum 
     * and updates the current position to whatever position was passed in. - copy pasted from documentation 
     * @param position
     */
    public void setIntakePosition(IntakePosition position) {
        _intakePosition = position;
        setIntakePositionWithAngle(position.getAngle());
    }

   // set roller motor to the speed for acquiring 
    public void setRollerMotorSpeedAcquire() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE_ACQUIRE_SPEED);
    }

   // sets roller motor to speed for spitting
    public void setRollerMotorSpeedSpit() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE_SPIT_SPEED);
    }
    // set roller motor speed to 0
    public void stopRollerMotor() {
        _intakeIO.setRollerMotorSpeed(0.05);
    }
/**
 * sets intake position to inputted angle 
 * also logs error if angle invalid :(
 * @param angle
 */
    public void setIntakePositionWithAngle(Double angle) {
        if (angle > IntakeConstants.MAX_INTAKE_ANGLE || angle < IntakeConstants.MIN_INTAKE_ANGLE) {
            // TODO: log an error, but don't throw exception
            return;
        }
        //_intakeIO.setTargetPositionAsDegrees(angle);
        _pivotPid.reset(getPivotAngle());
        _pivotPid.setGoal(angle);
    }

    /**
     * returns true if pivot is at its setpoint or false if not
     * @return
     */

    public boolean pivotAtSetpoint() {
        double pivotEncoderPositionDegrees = Units.rotationsToDegrees(_intakeInputs._pivotEncoderPositionDegrees);
        double targetPositionDegrees = _intakePosition.getAngle();
        return Math.abs(pivotEncoderPositionDegrees - targetPositionDegrees) <= IntakeConstants.INTAKE_PIVOT_DEADBAND;
    }
/**
 * detects if intake is holding note, returns true if it does have a note
 * @return
 */
    public boolean hasNote() {
        if (!_hasNote) {
            _hasNote = (_intakeInputs._rollerMotorCurrent > IntakeConstants.INTAKE_CURRENT_CUTOFF) && _intakeHasBeenRunning;
        }

        return _hasNote;
    }
  
   // resets hasNote
    public void resetHasNote() {
        _hasNote = false;
    }
/**
 * sets _intakeHasBeenRunning if intake has been running 
 * @param running
 */
    public void setIntakeHasBeenRunning(boolean running) {
        _intakeHasBeenRunning = running;
    }

/**
 * gets pivot angle (in degrees?)
 * @return
 */

    public double getPivotAngle() {
        return _intakeInputs._pivotEncoderPositionDegrees;
    }

    @AutoLogOutput(key = "Intake/TargetAngle")

    /**
     * returns target position 
     * @return
     */
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

        double angle = _intakeInputs._pivotEncoderPositionDegrees;
        _intakeVisualizer.update(angle);
        Logger.recordOutput("Intake/AngleDegrees", angle);
        Logger.recordOutput("Mechanism2D/Intake", _intakeVisualizer.getMechanism());
    }
}

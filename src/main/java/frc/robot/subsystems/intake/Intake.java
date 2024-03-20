package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged; checkstyle says this is redunant
import frc.robot.subsystems.shooter.ShooterConstants;

public class Intake extends SubsystemBase {

    public enum IntakePosition {
        SourcePickup(IntakeConstants.POSITION_SOURCE_PICKUP),
        GroundPickup(IntakeConstants.POSITION_GROUND_PICKUP),
        GroundSpit(IntakeConstants.POSITION_GROUND_SPIT),
        Stowed(IntakeConstants.POSITION_STOWED),
        AmpScore(IntakeConstants.POSITION_AMP_SCORE),
        
        SpeakerScore(IntakeConstants.POSITION_SPEAKER_SCORE);

        private final double _angle;

        /**
         * These enum represents a prefedined position of the intake. These are what we pass to
         * commands to set the intake positions
         * @param angle
         */
        private IntakePosition(double angle) {
            this._angle = angle;
        }

        /**
         * Return the angle associated with a given position. These angles are defined in IntakeConstants.java
         * @return
         */
        public double getAngle() {
            return this._angle;
        }
    }

    protected IntakePosition _intakePosition;
    private final IntakeCommandFactory _intakeCommandFactory = new IntakeCommandFactory(this);

    private final IntakeIO _intakeIO;
    private final IntakeIOInputsAutoLogged _intakeInputs = new IntakeIOInputsAutoLogged();

    private boolean _hasNote;
    private IntakeVisualizer _intakeVisualizer = new IntakeVisualizer();

    public Intake(IntakeIO intakeIO) {
        this._intakeIO = intakeIO;
        _hasNote = false;
        _intakePosition = IntakePosition.Stowed;                                    
        this.setIntakePosition(IntakePosition.Stowed);
    }

  

    /**
     * returns the position of the intake but not in degrees. 
     * Returns one of the known positions such as stowed, ground pickup, 
     * source, amp, etc
     * @return intakePosition
     */
    public IntakePosition getIntakePosition() {
        return this._intakePosition;
    }

    /**
     * Set intake angle based on a passed in position such as
     * amp, ground pickup, stowed, etc
     * @param position
     */
    public void setIntakePosition(IntakePosition position) {
        this._intakePosition = position;
        setIntakePositionWithAngle(position.getAngle());
    }

    /**
     * Sets the intake position based on the angle. If the angle is too small or big, return nothing.
     * @param angle
     */
    public void setIntakePositionWithAngle(Double angle) {
        if (angle > IntakeConstants.MAX_INTAKE_ANGLE || angle < IntakeConstants.MIN_INTAKE_ANGLE) {
            // TODO: log an error, but don't throw exception
            return;
        }
        _intakeIO.setTargetPositionAsDegrees(angle);
  
    }

    /**
     * Sets the roller motor to the acquire speed
     */
    public void setRollerMotorSpeedAcquire() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE_ACQUIRE_SPEED);
    }

    /**
     * Sets the roller motor to the acquire speed
     */
    public void setRollerMotorSpeed(double speed) {
        _intakeIO.setRollerMotorSpeed(speed);
    }

    /**
     * Sets the roller motor to the spit speed
     */
    public void setRollerMotorSpeedSpit() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE_SPIT_SPEED);
    }

    /**
     * Stops the roller motor
     */
    public void stopRollerMotor() {
        _intakeIO.setRollerMotorSpeed(IntakeConstants.INTAKE_STOP_SPEED);
    }

        public void setFeederMotorShootSpeed() {
        _intakeIO.setFeederMotorSpeed(ShooterConstants.FEEDER_SHOOT_SPEED);
    }

    public void setFeederMotorPickupSpeed() {
        _intakeIO.setFeederMotorSpeed(ShooterConstants.FEEDER_PICKUP_SPEED);
    }

    /**
     * Checks if the pivot is at the desired setpoint using encoder position.
     * @return boolean
     */
    public boolean pivotAtSetpoint() {
        double pivotEncoderPositionDegrees = _intakeInputs._pivotEncoderPositionDegrees;
        double targetPositionDegrees = _intakePosition.getAngle();
        boolean ret = Math.abs(pivotEncoderPositionDegrees - targetPositionDegrees) <= IntakeConstants.INTAKE_PIVOT_DEADBAND;
        Logger.recordOutput("Intake/IntakeAtSetpoint", ret);
        return ret;
    }

    /**
     * Checks if the intake has a note
     * @return _hasNote
     */
    public boolean hasNote() {
        if (!_hasNote) {
            _hasNote = _intakeInputs._leftLimitSwitchIsPushed || _intakeInputs._rightLimitSwitchIsPushed;
        }

        return _hasNote;
    }

    /**
     * Sets _hasNote to false
     */
    public void resetHasNote() {
        _hasNote = false;
    }

    /**
     * Set has note to true. Needed as a work around for autos for the time being.
     * 
     * Using this to tell the robot it has a note so it can do auto shoot
     */
    public void setHasNote() {
        _hasNote = true;
    }


    /**
     * Returns the pivot angle
     * @return pivotEncoderPositionDegrees
     */
    public double getPivotAngle() {
        return _intakeInputs._pivotEncoderPositionDegrees;
    }

    /**
     * Returns the target position
     * @return intakePosition as an angle (double)
     */
    public double getTargetPosition(){
        return this._intakePosition.getAngle();
    }

    /**
     * this is how we call commands to run the intake. to call a command use 
     * "intake.buildCommand.COMMANDTORUN(COMMAND ARGUMENTS)". replace command
     * to run with a command from {@link IntakeCommandFactory} and replace 
     * COMMAND ARGUMENTS with the arguments needed for that command, which could be empty.
     * @return
     */
    public IntakeCommandFactory buildCommand() {
        return this._intakeCommandFactory;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _intakeIO.updateInputs(_intakeInputs);
        Logger.processInputs("Intake", _intakeInputs);

        //smart dashboard testing
        SmartDashboard.putNumber("Intake Current Pivot Angle", _intakeInputs._pivotEncoderPositionDegrees);
        SmartDashboard.putNumber("Intake Desired Pivot Angle", _intakePosition.getAngle());
        SmartDashboard.putBoolean("Intake Has Note", _hasNote);

        // VISUALIZATION
        double angle = _intakeInputs._pivotEncoderPositionDegrees;
        _intakeVisualizer.update(angle);

        // LOGGING
        Logger.recordOutput("Intake/Current-Pivot-Angle", _intakeInputs._pivotEncoderPositionDegrees);
        Logger.recordOutput("Intake/Desired-Pivot-Angle", _intakePosition.getAngle());
        Logger.recordOutput("Intake/Has-Note", _hasNote);
    }
}

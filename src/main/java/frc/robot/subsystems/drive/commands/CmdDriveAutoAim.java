package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class CmdDriveAutoAim extends Command {
    private final Drive _drive;

    private final DoubleSupplier _translationXSupplier;
    private final DoubleSupplier _translationYSupplier;
    private PIDController _angleController;

    /**
     * Auto Rotate the robot to a specified angle. This command will still allow the
     * robot to drive while it rotates.
     * This command takes away rotation control from the driver until its finished.
     * 
     * @param drivetrainSubsystem  the drive subsytem
     * @param translationXSupplier a double supplier for the x movement
     * @param translationYSupplier a souble supplier for the y movement
     * @param angleSetpoint        the target angle of the robot heading
     */
    public CmdDriveAutoAim(Drive drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier) {

        this._drive = drivetrainSubsystem;
        this._translationXSupplier = translationXSupplier;
        this._translationYSupplier = translationYSupplier;

        this._angleController = new PIDController(DriveConstants.AUTO_ROTATE_P, DriveConstants.AUTO_ROTATE_I,
                DriveConstants.AUTO_ROTATE_D);

        this._angleController.setTolerance(DriveConstants.AUTO_ROTATE_TOLERANCE);
        this._angleController.enableContinuousInput(-180, 180);

        addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Calc the current angle to the speaker

        // inputModulus will wrap the value to between -180 and 180. This combine with
        // using enableContinuousInput on the PID Controller
        // means that our robot will always take the shortest path to the angle.
        // copied this from windham

        // we add 180 because the intake is the front of the robot and we want the shooter to
        // face the speaker not the intake.
        double currentAngleDegrees = _drive.getRotation().getDegrees() + 180;
        double desiredAngleDegrees =  _drive.calcAngleToSpeaker();

        this._angleController.setSetpoint(desiredAngleDegrees);
        double rotationVal = this._angleController.calculate(
                (MathUtil.inputModulus(currentAngleDegrees, -180, 180)));


        Logger.recordOutput("Drive/autoaim/rotationValue", rotationVal);
        Logger.recordOutput("Drive/autoaim/autorotatedesiredDegrees", desiredAngleDegrees);
        Logger.recordOutput("Drive/autoaim/autorotatedactualDegrees", currentAngleDegrees);

        // this is what drives the robot
        // drive the robot based on the calculations from above
        // _drive.runVelocity(
        //         _drive.transformJoystickInputsToChassisSpeeds(
        //         _translationXSupplier.getAsDouble(), 
        //         _translationYSupplier.getAsDouble(),
        //         rotationVal, true));     
        _drive.runVelocity(_drive.transformJoystickInputsToChassisSpeeds(
            _translationXSupplier.getAsDouble(), 
            _translationYSupplier.getAsDouble(), 
            rotationVal, true));
        
        Logger.recordOutput("Drive/autoaim/isReady", _angleController.atSetpoint());
        Logger.recordOutput("Drive/autoaim/isEnabled", true);
    }
    

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // when we finish set the rotation to 0 but keep driving
        _drive.runVelocity(_drive.transformJoystickInputsToChassisSpeeds(
            _translationXSupplier.getAsDouble(), 
            _translationYSupplier.getAsDouble(),
            0, false));
        Logger.recordOutput("Drive/autoaim/isReady", false);
        Logger.recordOutput("Drive/autoaim/isEnabled", false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return DriverStation.isAutonomousEnabled() && _angleController.atSetpoint();
    }
}

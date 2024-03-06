package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class CmdDriveRotateAboutSpeaker extends Command {
    private final Drive _drive;

    private final DoubleSupplier _translationXSupplier;
    private final DoubleSupplier _translationYSupplier;
    private PIDController _angleController;

    final double _autoRotateP = 0.13;
    final double _autoRotateI = 0.003;
    final double _autoRotateD = 0.00075;
    final double _autoRotateTolerance = 3.0;

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
    public CmdDriveRotateAboutSpeaker(Drive drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier) {

        this._drive = drivetrainSubsystem;
        this._translationXSupplier = translationXSupplier;
        this._translationYSupplier = translationYSupplier;

        this._angleController = new PIDController(this._autoRotateP, this._autoRotateI,
                this._autoRotateD);

        this._angleController.setTolerance(this._autoRotateTolerance);
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
        this._angleController.setSetpoint(_drive.calcAngleToSpeaker() - 90);

        // inputModulus will wrap the value to between -180 and 180. This combine with
        // using enableContinuousInput on the PID Controller
        // means that our robot will always take the shortest path to the angle.
        // copied this from windham
        double rotationVal = this._angleController.calculate(
                (MathUtil.inputModulus(this._drive.getGyroscopeRotation().getDegrees(), -180, 180)));
        Logger.recordOutput("Drive/autoaim/rotationValue", rotationVal);
        Logger.recordOutput("Drive/autoaim/angleToSpeaker", _drive.calcAngleToSpeaker());

        this._drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        _translationXSupplier.getAsDouble(),
                        _translationYSupplier.getAsDouble(),
                        rotationVal,
                        _drive.getGyroscopeRotation()));
      
                Logger.recordOutput("Drive/autoaim/autorotatedesiredDegrees", _drive.calcAngleToSpeaker() - 90);
                Logger.recordOutput("Drive/autoaim/autorotatedactualDegrees", _drive.getGyroscopeRotation().getDegrees());
    }
    

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // when we finish set the rotation to 0 but keep driving
        this._drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        _translationXSupplier.getAsDouble(),
                        _translationYSupplier.getAsDouble(),
                        0,
                        _drive.getGyroscopeRotation()));

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

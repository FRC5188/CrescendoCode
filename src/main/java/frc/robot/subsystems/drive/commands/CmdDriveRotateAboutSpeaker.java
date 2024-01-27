package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class CmdDriveRotateAboutSpeaker extends Command {
    private final Drive m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private PIDController m_angleController;

    // windham gains were 0.02, 0.0, 0.0. It worked well for their week 1
    private final double AUTO_ROTATE_KP = 0.2;
    private final double AUTO_ROTATE_KI = 0.001;
    private final double AUTO_ROTATE_KD = 0.001;
    private final double AUTO_ROTATE_TOLERANCE = 3.0;

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

        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;

        this.m_angleController = new PIDController(this.AUTO_ROTATE_KP, this.AUTO_ROTATE_KI,
                this.AUTO_ROTATE_KD);
        
        this.m_angleController.setTolerance(this.AUTO_ROTATE_TOLERANCE);
        this.m_angleController.enableContinuousInput(-180, 180);

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
        this.m_angleController.setSetpoint(m_drivetrainSubsystem.calcAngleToSpeaker());
        // inputModulus will wrap the value to between -180 and 180. This combine with
        // using enableContinuousInput on the PID Controller
        // means that our robot will always take the shortest path to the angle.
        // copied this from windham
        double rotationVal = this.m_angleController.calculate(
                (MathUtil.inputModulus(this.m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), -180, 180)));

        this.m_drivetrainSubsystem.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        rotationVal,
                        m_drivetrainSubsystem.getGyroscopeRotation()));
       System.out.println("desired: " + m_drivetrainSubsystem.calcAngleToSpeaker() + " actual: " + m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // when we finish set the rotation to 0 but keep driving
        this.m_drivetrainSubsystem.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        0,
                        m_drivetrainSubsystem.getGyroscopeRotation()));

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

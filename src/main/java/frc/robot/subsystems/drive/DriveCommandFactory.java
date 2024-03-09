package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.commands.CmdDriveRotateAboutSpeaker;

public class DriveCommandFactory {
    private Drive _drive;

    public DriveCommandFactory(Drive drive) {
        this._drive = drive;
    }

    public Command stopWithX() {
        return new InstantCommand(
            this._drive::stopWithX,
            this._drive);
    }

    public Command rotateAboutSpeaker(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        return new CmdDriveRotateAboutSpeaker(
            _drive, 
            translationXSupplier, 
            translationYSupplier);
    }

    public Command resetRobotFront() {
        return new InstantCommand(
            () -> _drive.setPose(
                new Pose2d(_drive.getPose().getTranslation(), new Rotation2d(Math.PI))),
            _drive).ignoringDisable(true);
    }

    public Command resetRobotPoseSpeaker() {
        final double inchesFromSubwoofer = 39.0;
        final double robotWidth = 13.0 + 1.5;
        final Pose2d robotOnSubwoofer = new Pose2d(
            DriveConstants.RED_SPEAKER.getX() - inchesFromSubwoofer + robotWidth,
            DriveConstants.RED_SPEAKER.getY(),
            new Rotation2d(Math.PI));
        
        return new InstantCommand(
            () -> _drive.setPose(robotOnSubwoofer),
            _drive);
    }

    public Command drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        final double DEADBAND = 0.1;

        return new InstantCommand(
            () -> {
                // APPLY DEADBAND FOR JOYSTICKS
                final double linearMagnitude = MathUtil.applyDeadband(
                    Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                final Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                final double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                // SQUARE VALUES
                final double linearMagnitudeSquared = Math.pow(linearMagnitude, 2);
                final double omegaSquared = Math.copySign(omega * omega, omega);

                // CALCULATE NEW LINEAR VELOCITY
                final Translation2d linearVelocity = new Pose2d(
                    new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitudeSquared, 0.0, new Rotation2d()))
                        .getTranslation();
                
                // CONVERT TO FIELD RELATIVE SPEEDS & SEND COMMAND
                final boolean isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
                
                _drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * _drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * _drive.getMaxLinearSpeedMetersPerSec(),
                        omegaSquared * _drive.getMaxAngularSpeedRadPerSec(),
                        isFlipped ? _drive.getRotation().plus(new Rotation2d(Math.PI)) : _drive.getRotation()));
            }, _drive
        );
    }
}

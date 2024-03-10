package frc.robot.util.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public abstract class ManualOdometryManipulation {
    public Command resetOrientation(Drive drive){
        return Commands.runOnce(
            () -> drive.setPose(
                new Pose2d(
                    drive.getPose().getTranslation(), new Rotation2d(Math.PI))),
                    drive
            ).ignoringDisable(true);
    }

    public Command resetPositionRedSpeaker(Drive drive){
        final double INCHES_FROM_SUBWOOFER = 39.0;
        final double ROBOT_WIDTH_INCHES = 13.0 + 1.5; // 13 INCHES ROBOT WIDTH + 1.5 INCHES FOR BUMPER
        final Pose2d ROBOT_ON_RED_SPEAKER = new Pose2d(
                DriveConstants.RED_SPEAKER.getX() - Units.inchesToMeters(INCHES_FROM_SUBWOOFER + ROBOT_WIDTH_INCHES),
                DriveConstants.RED_SPEAKER.getY(),
                new Rotation2d(Math.PI));

        return Commands.runOnce(() -> drive.setPose(ROBOT_ON_RED_SPEAKER), drive).ignoringDisable(true);
    }
}

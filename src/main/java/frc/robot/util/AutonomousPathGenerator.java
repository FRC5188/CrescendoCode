package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class AutonomousPathGenerator {
    public enum FIELD_POSITION {
        BLUE_ALLIANCE_SPEAKER(new Pose2d(0, 0, new Rotation2d(0))),
        RED_ALLIANCE_SPEAKER(new Pose2d(0, 0, new Rotation2d(0))),

        BLUE_ALLIANCE_AMP(new Pose2d(0, 0, new Rotation2d(0))),
        RED_ALLIANCE_AMP(new Pose2d(0, 0, new Rotation2d(0))),

        BLUE_ALLIANCE_SOURCE(new Pose2d(0, 0, new Rotation2d(0))),
        RED_ALLIANCE_SOURCE(new Pose2d(0, 0, new Rotation2d(0))),

        BLUE_ALLIANCE_SHOOTING_POSITION_ONE(new Pose2d(0, 0, new Rotation2d(0))),
        RED_ALLIANCE_SHOOTING_POSITION_ONE(new Pose2d(0, 0, new Rotation2d(0))),

        BLUE_ALLIANCE_SHOOTING_POSITION_TWO(new Pose2d(0, 0, new Rotation2d(0))),
        RED_ALLIANCE_SHOOTING_POSITION_TWO(new Pose2d(0, 0, new Rotation2d(0))),

        BLUE_ALLIANCE_SHOOTING_POSITION_THREE(new Pose2d(0, 0, new Rotation2d(0))),
        RED_ALLIANCE_SHOOTING_POSITION_THREE(new Pose2d(0, 0, new Rotation2d(0))),

        CENTER_OF_FIELD(new Pose2d(0, 0, new Rotation2d(0)));

        private Pose2d _fieldPosition;

        private FIELD_POSITION(Pose2d fieldPosition) {
            this._fieldPosition = fieldPosition;
        }

        public Pose2d getPose() {
            return this._fieldPosition;
        }
    }

    // This should always be called before a command is run. 
    public static void configureBuilder(
        Supplier<Pose2d> poseSupplier, 
        Consumer<Pose2d> resetPose, 
        Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier, 
        Consumer<ChassisSpeeds> robotRelativeOutput, 
        BooleanSupplier shouldFlipPath, 
        Subsystem driveSubsystem
    ){
        AutoBuilder.configureHolonomic(
            poseSupplier, 
            resetPose, 
            robotRelativeSpeedsSupplier, 
            robotRelativeOutput, 
            new HolonomicPathFollowerConfig(
                0, // TODO: Replace with actual maximum module speed that should be set in constants.
                0, // TODO: Replace with actual radius that should be set in constants.
                new ReplanningConfig(true, true)), // This will account for errors in the path, and let the robot dynamically adjust for them. 
            shouldFlipPath, 
            driveSubsystem);
    }

    public static Command getRuntimePath(Pose2d currentRobotPosition, FIELD_POSITION targetFieldPosition) {

    }

    // TODO: We'll need a way of automatically detecting whether we'll run into the wall or center and account for that with our generated paths. 
}

package frc.robot.util.autonomous;

import com.fasterxml.jackson.databind.Module;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;


public abstract class AutonomousPathGenerator {
    public static void configure(
        Supplier<Pose2d> poseSupplier, 
        Consumer<Pose2d> resetPose, 
        Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier, 
        Consumer<ChassisSpeeds> robotRelativeOutput, 
        BooleanSupplier shouldFlipPath, 
        Subsystem driveSubsystem) {
        AutoBuilder.configureHolonomic(
            poseSupplier, 
            resetPose, 
            robotRelativeSpeedsSupplier, 
            robotRelativeOutput, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(0, 0, 0), //TODO: These are placeholder values, must be updtaed
                new PIDConstants(0, 0, 0),
                DriveConstants.MAX_LINEAR_SPEED, // Assumed that this is also maximum module speed.
                DriveConstants.DRIVE_BASE_RADIUS,
                new ReplanningConfig(true, true)
            ), 
            shouldFlipPath, 
            driveSubsystem);
    }

    public static Command getStaticPathWithName(String name) {
        return AutoBuilder.buildAuto(name);
    }

    public static Command getRuntimePath(Pose2d start, Pose2d end){
        return null; // Implement graph-based algorithm for finding path implementing the Dijsktra algorithm for solving.
    }
}

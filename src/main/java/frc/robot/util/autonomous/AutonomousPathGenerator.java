package frc.robot.util.autonomous;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;


public abstract class AutonomousPathGenerator {

    public static void configure(Drive drive, SwerveDriveKinematics kinematics, SwerveModuleState[] moduleStates) {
        AutoBuilder.configureHolonomic(
            drive::getPose,
            drive::setPose, 
            () -> kinematics.toChassisSpeeds(moduleStates), 
            drive::runVelocity, 
            new HolonomicPathFollowerConfig(
                DriveConstants.MAX_LINEAR_SPEED, 
                DriveConstants.DRIVE_BASE_RADIUS, 
                new ReplanningConfig()), 
            () -> DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red, 
            drive);

        Pathfinding.setPathfinder(new LocalADStar());

        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
              Logger.recordOutput(
                  "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
              Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });
    }

    /**
     * @param name Name of the file which has the path.
     * @return Command which will execute the static path.
     */
    public static Command getStaticPath(String name) {
        return AutoBuilder.buildAuto(name);
    }

    /**
     * @param targetPose Pose which you want to navigate toward.
     * @return Command which goes to that pose.
     */
    public static Command getRuntimePath(Pose2d targetPose) {
        final double endVelocityGoal = 0.0; // We want to be stopped at the end of our path.

        final double maxLinearAcceleration = 4.0;
        final double maxAngularAcceleration = Units.degreesToRadians(720);

        return AutoBuilder.pathfindToPose(
            targetPose,
            new PathConstraints(
                DriveConstants.MAX_LINEAR_SPEED, 
                maxLinearAcceleration, 
                DriveConstants.MAX_ANGULAR_SPEED, 
                maxAngularAcceleration),
            endVelocityGoal
        );
    }
}

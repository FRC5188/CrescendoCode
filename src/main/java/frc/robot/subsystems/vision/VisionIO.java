package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.HardwareConstants;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        // If we don't see any tags then in avoiding a NullPointerException we'll give it a new Pose2d(0, 0, new Rotation2d(0)).
        public Pose2d[] _poses = new Pose2d[HardwareConstants.NUMBER_OF_CAMERAS];
        public double[] _timestamps = new double[HardwareConstants.NUMBER_OF_CAMERAS];
        public boolean[] _hasPose = new boolean[HardwareConstants.NUMBER_OF_CAMERAS];

        // (2024-2025 TODO): You should log all the tags that are being used in the estimation. You can do this through a boolean array with the indexes
        // representing the tags and T/F representing whether they're being used or not. You can see what tags are being used by using the {@link PhotonPoseEstimator}.
        // Good luck! :)
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    /**
     * Boolean whether we should use the estimated pose given by the camera through filtering out the poses that are more than 
     * 2.0m away from the current pose that we're in which is physically impossible for our robot to do.
     * @param estimatedRobotPose Estimated Pose from Vision
     * @param currentPose Current Pose of the Robot
     * @return Whether we should use this estimation or not.
     */
    public static boolean shouldUsePoseEstimation(Pose2d estimatedRobotPose, Pose2d currentPose){
        final double MAXIMUM_ALLOWED_DISTANCE_BETWEEN_POSES = 2.0;

        return !(Math.abs(PhotonUtils.getDistanceToPose(currentPose, estimatedRobotPose)) > MAXIMUM_ALLOWED_DISTANCE_BETWEEN_POSES);
    }
}
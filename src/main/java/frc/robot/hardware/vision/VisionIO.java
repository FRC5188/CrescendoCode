package frc.robot.hardware.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
    
    @AutoLog
    public static class VisionIOInputs {
        // While the variables above are for the cameras themselves the ones below are after they have been combined into one estimate.
        public Pose2d _combinedPose;
        public double _combinedRotationRadians;
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void setOdometryReferenceEstimator(SwerveDrivePoseEstimator inputEstimationFromOdometry) {} // This is passed in and then we use vision to account for error then the current estimated pose is put into the table.

    public SwerveDrivePoseEstimator getUpdatedEstimation();
}

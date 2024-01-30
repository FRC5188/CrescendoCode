package frc.robot.hardware.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.hardware.HardwareConstants;

import frc.robot.hardware.vision.VisionIO.VisionIOInputs;

public class RealVisionIO implements VisionIO {
        private static final int NUMBER_OF_CAMERAS = 2;
        private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        private static final List<PhotonPoseEstimator> ESTIMATORS = createPhotonPoseEstimators();

        private static SwerveDrivePoseEstimator _poseEstimatorFromOdometry;

    public void updateInputs(VisionIOInputs inputs) {
        
        Map<Pose3d, Double> cameraPoseTimestampMap = getPoseAndTimestampMap(ESTIMATORS);

        cameraPoseTimestampMap.forEach(
                (pose, timestamp) -> {
                        if (!pose.equals(new Pose3d(0, 0, 0, new Rotation3d(0,0,  0)))) {
                                _poseEstimatorFromOdometry.addVisionMeasurement(
                                        pose.toPose2d(),
                                        timestamp);
                        }
                }
        );

        inputs._combinedPose = _poseEstimatorFromOdometry.getEstimatedPosition();
    }

    // Note: This has to be run in the periodic() for Odometry.
    public void setOdometryReferenceEstimator(SwerveDrivePoseEstimator poseEstimatorFromOdometry) {
        _poseEstimatorFromOdometry = poseEstimatorFromOdometry;
    }

    // Note: This must be run after you set the new estimation and update inputs.
    public SwerveDrivePoseEstimator getUpdatedEstimation() {
        return _poseEstimatorFromOdometry;
    }

    private static List<PhotonPoseEstimator> createPhotonPoseEstimators() {
        // Creates the cameras.
        List<PhotonCamera> cameras = new ArrayList<>();
        for (int i = 0; i < NUMBER_OF_CAMERAS; i++) {
            cameras.add(new PhotonCamera("photoncamera_" + (i + 1)));
        }

        // Creates the estimators for each camera.
        List<PhotonPoseEstimator> estimators = new ArrayList<>();
        for (int i = 0; i < NUMBER_OF_CAMERAS; i++) {
            estimators.add(new PhotonPoseEstimator(
                    FIELD_LAYOUT,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    cameras.get(i),
                    HardwareConstants.ComponentTransformations._cameraOnePosition));
        }
        estimators.forEach( // Sets the fallback strategy for each estimator to be lowest ambiguity.
                (estimator) -> estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
        );

        return estimators;
    }

    private static Map<Pose3d, Double> getPoseAndTimestampMap(List<PhotonPoseEstimator> estimators) {
        Map<Pose3d, Double> map = new HashMap<>();
        for (PhotonPoseEstimator estimator : estimators) {
                Optional<EstimatedRobotPose> pose = estimator.update();
                if (pose.isEmpty()) {
                        map.put(new Pose3d(0, 0, 0, new Rotation3d(0,0,  0)), 0.0);
                } else {
                        map.put(pose.get().estimatedPose, Timer.getFPGATimestamp());
                }
        }
        return map;
    }
}

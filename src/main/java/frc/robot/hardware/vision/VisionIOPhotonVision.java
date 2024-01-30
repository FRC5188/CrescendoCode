package frc.robot.hardware.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.hardware.HardwareConstants;
import frc.robot.hardware.vision.VisionIO.VisionIOInputs;

public class VisionIOPhotonVision implements VisionIO{
        // TODO: Make a function that returns the camera with the best target based on ambiguity. 

        private static final int NUMBER_OF_CAMERAS = 2;
        private static final AprilTagFieldLayout _fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        private static final List<PhotonPoseEstimator> _estimators = createPhotonPoseEstimators();

        private static SwerveDrivePoseEstimator _poseEstimatorFromOdometry;

    public void updateInputs(VisionIOInputs inputs) {
        
        Map<Pose3d, Double> cameraPoseTimestampMap = getPoseAndTimestampMap(_estimators);

        cameraPoseTimestampMap.forEach(
                (pose, timestamp) -> {
                        if (!pose.equals(new Pose3d(0, 0, 0, new Rotation3d(0,0,  0))))
                        {
                                _poseEstimatorFromOdometry.addVisionMeasurement(
                                        pose.toPose2d(),
                                        timestamp);
                        }
                }
        );

        inputs._combinedPose = _poseEstimatorFromOdometry.getEstimatedPosition();
        inputs._combinedRotationRadians = _poseEstimatorFromOdometry.getEstimatedPosition().getRotation().getRadians();
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
        List<PhotonCamera> _cameras = new ArrayList<>();
        for (int i = 0; i < NUMBER_OF_CAMERAS; i++) {
            _cameras.add(new PhotonCamera("photoncamera_" + (i + 1)));
        }

        // Creates the estimators for each camera.
        List<PhotonPoseEstimator> _estimators = new ArrayList<>();
        for (int i = 0; i < NUMBER_OF_CAMERAS; i++) {
            _estimators.add(new PhotonPoseEstimator(
                    _fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    _cameras.get(i),
                    HardwareConstants.ComponentTransformations._cameraOnePosition));
        }
        _estimators.forEach( // Sets the fallback strategy for each estimator to be lowest ambiguity.
                (estimator) -> estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
        );

        return _estimators;
    }

    private static Map<Pose3d, Double> getPoseAndTimestampMap(List<PhotonPoseEstimator> estimators) {
        Map<Pose3d, Double> _map = new HashMap<>();
        for (PhotonPoseEstimator estimator : estimators) {
                Optional<EstimatedRobotPose> pose = estimator.update();
                if (pose.isEmpty()) {
                        _map.put(new Pose3d(0, 0, 0, new Rotation3d(0,0,  0)), 0.0);
                }
                else {
                        _map.put(pose.get().estimatedPose, Timer.getFPGATimestamp());
                }
        }
        return _map;
    }
}

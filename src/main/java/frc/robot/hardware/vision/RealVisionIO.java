package frc.robot.hardware.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.hardware.HardwareConstants;

public class RealVisionIO implements VisionIO {
        private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        private static final List<PhotonCamera> CAMERAS = createCameras();
        private static final List<PhotonPoseEstimator> ESTIMATORS = createPoseEstimators(CAMERAS);

        public void updateInputs(VisionIOInputs inputs) {
        
                for (int n = 0; n < HardwareConstants.NUMBER_OF_CAMERAS; n++) {
                        Optional<EstimatedRobotPose> estimatedPose = ESTIMATORS.get(n).update(CAMERAS.get(n).getLatestResult());

                        if (estimatedPose.isPresent()){
                                inputs._poseList[n] = estimatedPose.get().estimatedPose;
                                inputs._timestampList[n] = Timer.getFPGATimestamp();
                                System.out.println("Pose: " + inputs._poseList[n] + " Timestamp: " + inputs._timestampList[n]);
                        }
                        else {
                                inputs._poseList[n] = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
                                inputs._timest0ampList[n] = 0;
                        }
                }
        }

        private static List<PhotonCamera> createCameras() {
                // Creates the cameras.
                List<PhotonCamera> cameras = new ArrayList<>();
                for (int i = 0; i < HardwareConstants.NUMBER_OF_CAMERAS; i++) {
                        cameras.add(new PhotonCamera("photoncamera_" + (i + 1)));
                }

                return cameras;
        }

        private static List<PhotonPoseEstimator> createPoseEstimators(List<PhotonCamera> cameras) {
                List<PhotonPoseEstimator> estimators = new ArrayList<>();
                for (int i = 0; i < HardwareConstants.NUMBER_OF_CAMERAS; i++) {
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
}

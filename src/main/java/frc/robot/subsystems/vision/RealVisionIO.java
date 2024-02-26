package frc.robot.subsystems.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.HardwareConstants;

public class RealVisionIO implements VisionIO {
        private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        private static final PhotonPoseEstimator[] ESTIMATORS = createPoseEstimators();

        public void updateInputs(VisionIOInputs inputs) {
        
                for (int n = 0; n < HardwareConstants.NUMBER_OF_PHOTONVISION_CAMERAS; n++) {
                        Optional<EstimatedRobotPose> estimatedPose = ESTIMATORS[n].update();

                        if (estimatedPose.isPresent()){
                                inputs._poses[n] = estimatedPose.get().estimatedPose.toPose2d();
                                inputs._timestamps[n] = estimatedPose.get().timestampSeconds;
                                inputs._hasPose[n] = true;
                        } else {
                                inputs._hasPose[n] = false;
                                inputs._poses[n] = new Pose2d();
                        }
                }

                for (int n = 0; n < HardwareConstants.NUMBER_OF_LIMELIGHT_CAMERAS; n++) {
                        
                }
        }

        private static PhotonPoseEstimator[] createPoseEstimators() {
                final PhotonPoseEstimator[] estimators = new PhotonPoseEstimator[HardwareConstants.NUMBER_OF_PHOTONVISION_CAMERAS];
                for (int i = 0; i < HardwareConstants.NUMBER_OF_PHOTONVISION_CAMERAS; i++) {
                        estimators[i] = new PhotonPoseEstimator(
                                FIELD_LAYOUT,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                new PhotonCamera("photonvision" + (i + 1)),
                                HardwareConstants.ComponentTransformations._cameraPosition[i]);
                        
                        estimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                }
                return estimators;
        }
}

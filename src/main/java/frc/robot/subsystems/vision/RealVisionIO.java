package frc.robot.subsystems.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.HardwareConstants;

public class RealVisionIO implements VisionIO {
        private static final double AMBIGUITY_CUTOFF = 0.1;

        private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        private static final PhotonPoseEstimator[] ESTIMATORS = createPoseEstimators();

        public void updateInputs(VisionIOInputs inputs) {
        
                for (int n = 0; n < HardwareConstants.NUMBER_OF_CAMERAS; n++) {
                        Optional<EstimatedRobotPose> estimatedPose = ESTIMATORS[n].update();




                        if (estimatedPose.isPresent()){

                                // We'll check to make sure that the poses that are referenced are not too ambiguous.
                                estimatedPose = Optional.of(getPoseWithAmbiguityCutoff(estimatedPose.get()));

                                // Now that we've removed some of the ambiguous poses, we can check to see if we have any poses left.
                                // If not then we'll just pretend that we don't have a pose.
                                if (estimatedPose.get().targetsUsed.size() == 0){
                                        inputs._hasPose[n] = false;
                                        inputs._poses[n] = new Pose2d();
                                        continue; // Move On.
                                }

                                // This is what happens whenever we have a pose that is not too ambiguous.
                                inputs._poses[n] = estimatedPose.get().estimatedPose.toPose2d();
                                inputs._timestamps[n] = estimatedPose.get().timestampSeconds;
                                inputs._hasPose[n] = true;
                        } else {
                                inputs._hasPose[n] = false;
                                inputs._poses[n] = new Pose2d();
                        }
                }
        }

        private static PhotonPoseEstimator[] createPoseEstimators() {
                final PhotonPoseEstimator[] estimators = new PhotonPoseEstimator[HardwareConstants.NUMBER_OF_CAMERAS];
                for (int i = 0; i < HardwareConstants.NUMBER_OF_CAMERAS; i++) {
                        estimators[i] = new PhotonPoseEstimator(
                                FIELD_LAYOUT,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                new PhotonCamera("photoncamera_" + (i + 1)),
                                HardwareConstants.ComponentTransformations._cameraPosition[i]);

                        estimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                }
                return estimators;
        }

        /**
         * Will use the constant ambiguity cutoff in removing any poses that are too ambiguous.
         * @param pose Estimated Robot Pose from Vision
         * @return Estimated Robot Pose with Ambigious Poses Removed
         */
        private static EstimatedRobotPose getPoseWithAmbiguityCutoff(EstimatedRobotPose pose){
                for (int i = 0; i < pose.targetsUsed.size(); i++) {
                        PhotonTrackedTarget target = pose.targetsUsed.get(i);

                        if (target.getPoseAmbiguity() > AMBIGUITY_CUTOFF){
                                pose.targetsUsed.remove(target);
                        }
                }

                return pose;
        }
}
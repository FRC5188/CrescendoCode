package frc.robot.subsystems.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class RealVisionIO implements VisionIO {
        private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        private static final PhotonCamera[] CAMERAS = createCameras();
        private static final PhotonPoseEstimator[] ESTIMATORS = createPoseEstimators();

        // Updated Every Cycle.
        public void updateInputs(VisionIOInputs inputs) {
        
                for (int n = 0; n <VisionConstants.Software.NUMBER_OF_CAMERAS; n++) {
                        final PhotonPipelineResult results = CAMERAS[n].getLatestResult();
                        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

                        // Here we're filtering out the tags that have ambiguous poses that we don't really want to use.
                        if (results.hasTargets() && (results.targets.size() > 1 || results.targets.get(0).getPoseAmbiguity() < VisionConstants.Software.AMBIGUITY_CUTOFF)){
                                estimatedPose = ESTIMATORS[n].update(results);
                        }

                        // (2024-2025 TODO): Filter out poses that are outside the field or that are more than 2.0m away from the current position of the robot.
                        // While we shouldn't to have to worry much about this with the cut-off filter it's just another way we can keep improving this part of 
                        // our robot. :)

                        // If we have an estimated pose then we'll use it...
                        if (estimatedPose.isPresent()){
                                inputs._hasPose[n] = true;
                                inputs._poses[n] = estimatedPose.get().estimatedPose.toPose2d();
                                inputs._timestamps[n] = estimatedPose.get().timestampSeconds;
                                continue;
                        }

                        // If not then we don't see any tags or we shouldn't use the tags that we do see so we'll not do anything.
                        inputs._hasPose[n] = false;
                        inputs._poses[n] = new Pose2d();
                        inputs._timestamps[n] = results.getTimestampSeconds();     
                        continue;    
                }
        }

        /**
         * @return Array of {@link PhotonPoseEstimator PhotonPoseEstimators} for each {@link PhotonCamera PhotonCamera}.
         */
        private static PhotonPoseEstimator[] createPoseEstimators() {
                // Whenever using Multi-Tag PNP make sure you select the option on the Photon Camera itself. You can do this in the 
                // PhotonVision GUI.
                final PhotonPoseEstimator[] estimators = new PhotonPoseEstimator[VisionConstants.Software.NUMBER_OF_CAMERAS];
                for (int i = 0; i <VisionConstants.Software.NUMBER_OF_CAMERAS; i++) {
                        estimators[i] = new PhotonPoseEstimator(
                                FIELD_LAYOUT,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                CAMERAS[i],
                                VisionConstants.Physical._cameraPosition[i]);

                        estimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                }
                return estimators;
        }

        /**
         * @param tagNumber AprilTag ID
         * @return Pose of AprilTag on the Field.
         */
        public static Pose3d getPoseOfTagFromID(int tagNumber){
                return FIELD_LAYOUT.getTagPose(tagNumber).get();
        }

        /**
         * @return Array of the {@link PhotonCamera PhotonCameras} connected to the Co-Processor.
         */
        private static PhotonCamera[] createCameras(){
                PhotonCamera[] cameras = new PhotonCamera[VisionConstants.Software.NUMBER_OF_CAMERAS];

                for (int i = 0; i < VisionConstants.Software.NUMBER_OF_CAMERAS; i++){
                        cameras[i] = new PhotonCamera("photoncamera_" + (i + 1));
                }

                return cameras;
        }
}
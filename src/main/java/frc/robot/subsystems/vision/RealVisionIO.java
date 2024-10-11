package frc.robot.subsystems.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.HardwareConstants;

public class RealVisionIO implements VisionIO {
        private static final double AMBIGUITY_CUTOFF = 0.2;

        private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        private static final PhotonCamera[] CAMERAS = createCameras();
        private static final PhotonPoseEstimator[] ESTIMATORS = createPoseEstimators();

        public void updateInputs(VisionIOInputs inputs) {
        
                for (int n = 0; n < HardwareConstants.NUMBER_OF_CAMERAS; n++) {
                        final PhotonPipelineResult results = CAMERAS[n].getLatestResult();
                        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

                        // The first thing we'll do is filter out the bad tags from our cameras.
                        if (results.hasTargets() && (results.targets.size() > 1 || results.targets.get(0).getPoseAmbiguity() < AMBIGUITY_CUTOFF)){
                                //double imageCaptureTime = results.getTimestampSeconds();
                                // DON'T IF YOU ACTUALLY NEED THIS, BUT WE'LL SEE - MITCHELL :)
                                estimatedPose = ESTIMATORS[n].update(results);
                        }

                        if (estimatedPose.isPresent()){
                                inputs._hasPose[n] = true;
                                inputs._poses[n] = estimatedPose.get().estimatedPose.toPose2d();
                                inputs._timestamps[n] = estimatedPose.get().timestampSeconds;
                                continue;
                        }

                        // If we can't do anything then just set it to null so at least we're not breaking everything.
                        inputs._hasPose[n] = false;
                        inputs._poses[n] = new Pose2d();
                        inputs._timestamps[n] = results.getTimestampSeconds();     
                        continue;    
                }
        }
        private static PhotonPoseEstimator[] createPoseEstimators() {
                final PhotonPoseEstimator[] estimators = new PhotonPoseEstimator[HardwareConstants.NUMBER_OF_CAMERAS];
                for (int i = 0; i < HardwareConstants.NUMBER_OF_CAMERAS; i++) {
                        estimators[i] = new PhotonPoseEstimator(
                                FIELD_LAYOUT,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                CAMERAS[i],
                                HardwareConstants.ComponentTransformations._cameraPosition[i]);

                        estimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                }
                return estimators;
        }

        public static Pose3d getPoseOfTagFromID(int i){
                return FIELD_LAYOUT.getTagPose(i).get();
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

        private static PhotonCamera[] createCameras(){
                PhotonCamera[] cameras = new PhotonCamera[HardwareConstants.NUMBER_OF_CAMERAS];

                for (int i = 0; i < HardwareConstants.NUMBER_OF_CAMERAS; i++){
                        cameras[i] = new PhotonCamera("photoncamera_" + (i + 1));
                }

                return cameras;
        }
}
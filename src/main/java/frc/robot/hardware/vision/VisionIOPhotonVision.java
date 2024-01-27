package frc.robot.hardware.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.hardware.HardwareConstants;
import frc.robot.hardware.vision.VisionIO.VisionIOInputs;

public class VisionIOPhotonVision {
    private static final PhotonCamera CAMERA_ONE = new PhotonCamera(HardwareConstants.ComponentTransformations.CAMERA_ONE_NAME);
    private static final PhotonCamera CAMERA_TWO = new PhotonCamera(HardwareConstants.ComponentTransformations.CAMERA_TWO_NAME);
    
    private static AprilTagFieldLayout _fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

    private static final PhotonPoseEstimator CAMERA_ONE_ESTIMATION = new PhotonPoseEstimator(
        _fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        CAMERA_ONE,
        HardwareConstants.ComponentTransformations._cameraOnePosition);

    private static final PhotonPoseEstimator CAMERA_TWO_ESTIMATION = new PhotonPoseEstimator(
        _fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        CAMERA_TWO,
        HardwareConstants.ComponentTransformations._cameraTwoPosition);

    static {
        // If we can't use multiple tags then just use the target with the lowest ambiguity.
        CAMERA_ONE_ESTIMATION.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        CAMERA_TWO_ESTIMATION.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    private double _cameraOneTimestamp;
    private double _cameraTwoTimestamp;

    private List<PhotonTrackedTarget> _cameraOneTargets;
    private List<PhotonTrackedTarget> _cameraTwoTargets;

    private boolean _doesCameraOneHaveTarget = false;
    private boolean _doesCameraTwoHaveTarget = false;

    public void updateInputs(VisionIOInputs inputs) {
        PhotonPipelineResult cameraOneResult = CAMERA_ONE.getLatestResult();
        //PhotonPipelineResult cameraTwoResult = 
    }

    public void setOdometryReferenceEstimator(SwerveDrivePoseEstimator inputEstimationFromOdometry) {

    } 
}   

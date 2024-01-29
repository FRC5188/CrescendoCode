package frc.robot.hardware.vision;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.hardware.HardwareConstants;
import frc.robot.hardware.vision.VisionIO.VisionIOInputs;

public class VisionIOPhotonVision {
    private static final PhotonCamera CAMERA_ONE = new PhotonCamera(
            HardwareConstants.ComponentTransformations.CAMERA_ONE_NAME);
    private static final PhotonCamera CAMERA_TWO = new PhotonCamera(
            HardwareConstants.ComponentTransformations.CAMERA_TWO_NAME);

    private static AprilTagFieldLayout _fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

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
        // If we can't use multiple tags then just use the target with the lowest
        // ambiguity.
        CAMERA_ONE_ESTIMATION.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        CAMERA_TWO_ESTIMATION.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    private double _cameraOneTimestamp;
    private double _cameraTwoTimestamp;

    private List<PhotonTrackedTarget> _cameraOneTargets;
    private List<PhotonTrackedTarget> _cameraTwoTargets;

    private Pose3d _cameraOneEstimatedPose;
    private Pose3d _cameraTwoEstimatedPose;

    private boolean _doesCameraOneHaveTarget = false;
    private boolean _doesCameraTwoHaveTarget = false;

    private SwerveDrivePoseEstimator _poseEstimatorFromOdometry;

    public void updateInputs(VisionIOInputs inputs) {
        PhotonPipelineResult cameraOneResult = CAMERA_ONE.getLatestResult();
        PhotonPipelineResult cameraTwoResult = CAMERA_TWO.getLatestResult();

        _cameraOneTimestamp = Timer.getFPGATimestamp();
        _cameraTwoTimestamp = Timer.getFPGATimestamp();

        _cameraOneTargets = cameraOneResult.getTargets();
        _cameraTwoTargets = cameraTwoResult.getTargets();

        _doesCameraOneHaveTarget = checkValidResult(_cameraOneTargets) && !_cameraOneTargets.isEmpty();
        _doesCameraTwoHaveTarget = checkValidResult(_cameraTwoTargets) && !_cameraTwoTargets.isEmpty();

        if (_doesCameraOneHaveTarget && _doesCameraTwoHaveTarget) {
            // We can use both cameras for our calculations
            _cameraOneEstimatedPose = CAMERA_ONE_ESTIMATION.update().get().estimatedPose;
            _cameraTwoEstimatedPose = CAMERA_TWO_ESTIMATION.update().get().estimatedPose;
            _poseEstimatorFromOdometry.addVisionMeasurement(
                    _cameraOneEstimatedPose.toPose2d(),
                    _cameraOneTimestamp);

            _poseEstimatorFromOdometry.addVisionMeasurement(
                    _cameraTwoEstimatedPose.toPose2d(),
                    _cameraTwoTimestamp);
        } else if (_doesCameraOneHaveTarget) {
            // Then we'll just use camera one.
            _poseEstimatorFromOdometry.addVisionMeasurement(
                    _cameraOneEstimatedPose.toPose2d(),
                    _cameraOneTimestamp);
        } else if (_doesCameraTwoHaveTarget) {
            // Then we'll just use camera two.
            _poseEstimatorFromOdometry.addVisionMeasurement(
                    _cameraTwoEstimatedPose.toPose2d(),
                    _cameraTwoTimestamp);
        } 
        // We can't use either. I don't know fix it.
        // Right now we'll just not do anything and hope it works :).

        inputs._cameraOneX = (_doesCameraOneHaveTarget) ? (_cameraOneEstimatedPose.getX()) : (-1.0);
        inputs._cameraOneY = (_doesCameraOneHaveTarget) ? (_cameraOneEstimatedPose.getY()) : (-1.0);
        inputs._cameraOneTimestamp = _cameraOneTimestamp;
        inputs._cameraOneMaxAmbiguity = _cameraOneTargets.stream().max(
                (a, b) -> Double.compare(a.getPoseAmbiguity(), b.getPoseAmbiguity()))
                .get().getPoseAmbiguity();
        inputs._camerOneMaxDistance = _cameraOneTargets.stream().max(
                (a, b) -> Double.compare(a.getBestCameraToTarget().getTranslation().getNorm(),
                        b.getBestCameraToTarget().getTranslation().getNorm()))
                .get().getPoseAmbiguity();
        inputs._cameraOneHasTarget = this._doesCameraOneHaveTarget;
        inputs._cameraOneRotationRadians = (_doesCameraOneHaveTarget) ? (_cameraOneEstimatedPose.getRotation().toRotation2d().getRadians()) : (-1.0);

        inputs._cameraTwoX = (_doesCameraTwoHaveTarget) ? (_cameraTwoEstimatedPose.getX()) : (-1.0);
        inputs._cameraTwoY = (_doesCameraTwoHaveTarget) ? (_cameraTwoEstimatedPose.getY()) : (-1.0);
        inputs._cameraTwoTimestamp = _cameraTwoTimestamp;
        inputs._cameraTwoMaxAmbiguity = _cameraTwoTargets.stream().max(
                (a, b) -> Double.compare(a.getPoseAmbiguity(), b.getPoseAmbiguity()))
                .get().getPoseAmbiguity(); // TODO: Not sure if this actually gets the max of the list but we'll see :).
        inputs._cameraTwoMaxDistance = _cameraTwoTargets.stream().max(
                (a, b) -> Double.compare(a.getBestCameraToTarget().getTranslation().getNorm(),
                        b.getBestCameraToTarget().getTranslation().getNorm()))
                .get().getPoseAmbiguity();
        inputs._cameraTwoHasTarget = this._doesCameraTwoHaveTarget;
        inputs._cameraOneRotationRadians = (_doesCameraOneHaveTarget) ? (_cameraOneEstimatedPose.getRotation().toRotation2d().getRadians()) : (-1.0);

        inputs._combinedX = _poseEstimatorFromOdometry.getEstimatedPosition().getX();
        inputs._combinedY = _poseEstimatorFromOdometry.getEstimatedPosition().getY();
        inputs._combinedRotationRadians = _poseEstimatorFromOdometry.getEstimatedPosition().getRotation().getRadians();
    }

    // Note: This has to be run in the periodic() for Odometry.
    public void setOdometryReferenceEstimator(SwerveDrivePoseEstimator poseEstimatorFromOdometry) {
        this._poseEstimatorFromOdometry = poseEstimatorFromOdometry;
    }

    // Note: This must be run after you set the new estimation and update inputs.
    public SwerveDrivePoseEstimator getUpdatedEstimation() {
        return this._poseEstimatorFromOdometry;
    }

    private boolean checkValidResult(List<PhotonTrackedTarget> result) {
        for (PhotonTrackedTarget target : result) {
            if (target.getFiducialId() > 8) { // TODO: This will need to be updated with new ID for tags.
                return false;
            }
        }
        return true;
    }
}

package frc.robot.hardware.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.hardware.HardwareConstants;
import frc.robot.hardware.vision.VisionIO.VisionIOInputs;

public class VisionIOPhotonVision {
    private static boolean cameraOneHasTarget;
    private static boolean cameraTwoHasTarget;

    private static Pose3d cameraOneEstimatedPosition;
    private static Pose3d cameraTwoEstimatedPosition;

    private static double cameraOneTimestamp;
    private static double cameraTwoTimestamp;

    private static Pose2d combinedEstimatedPosition;

    private static SwerveDrivePoseEstimator inputEstimationFromOdometry;

    private static AprilTagFieldLayout layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

    static final PhotonCamera CAMERA_ONE = new PhotonCamera(HardwareConstants.ComponentTransformations.CAMERA_ONE_NAME);
    static final PhotonCamera CAMERA_TWO = new PhotonCamera(HardwareConstants.ComponentTransformations.CAMERA_TWO_NAME);

    private static final PhotonPoseEstimator CAMERA_ONE_ESTIMATION = new PhotonPoseEstimator(
        layout,
        PoseStrategy.LOWEST_AMBIGUITY,
        CAMERA_ONE,
        HardwareConstants.ComponentTransformations._cameraOnePosition);

    private static final PhotonPoseEstimator CAMERA_TWO_ESTIMATION = new PhotonPoseEstimator(
        layout,
        PoseStrategy.LOWEST_AMBIGUITY, // TODO: Pretty sure we're changing this strategy later on. 
        CAMERA_TWO,
        HardwareConstants.ComponentTransformations._cameraTwoPosition);

    public void updateInputs(VisionIOInputs inputs) {
        // We need to run the actual update method for the cameras. 
        this.update(inputEstimationFromOdometry);

        inputs._cameraOneX = cameraOneEstimatedPosition.getTranslation().getX();
        inputs._cameraOneY = cameraOneEstimatedPosition.getTranslation().getY();
        inputs._cameraOneTimestamp = cameraOneTimestamp;

        inputs._cameraTwoX = cameraTwoEstimatedPosition.getTranslation().getX();
        inputs._cameraTwoY = cameraTwoEstimatedPosition.getTranslation().getY();
        inputs._cameraTwoTimestamp = cameraTwoTimestamp;

        inputs._combinedX = combinedEstimatedPosition.getTranslation().getX();
        inputs._combinedY = combinedEstimatedPosition.getTranslation().getY();

        inputs._targetAmbiguity = CAMERA_ONE_ESTIMATION.getLatestResult().getBestTarget().getAmbiguity();

        // TODO: Still need to do work here. This won't build right now. 
    }


    public void setOdometryReferenceEstimation(SwerveDrivePoseEstimator inputEstimationFromOdometry) {
        PhotonTrackedTarget cameraOneBestTarget = CAMERA_ONE.getLatestResult().getBestTarget();
        PhotonTrackedTarget cameraTwoBestTarget = CAMERA_TWO.getLatestResult().getBestTarget();


    private Pose3d getEstimatedRobotPose(SwerveDrivePoseEstimator inputEstimationFromOdometry, PhotonCamera camera, Transform3d cameraPos) throws NullPointerException {
        PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
        if (target != null) {
          return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
              layout.getTagPose(target.getFiducialId()).get(), cameraPos);
        } else {
          throw new NullPointerException("[ERROR]: Robot Cannot See Apriltag");
        }
    }

    private Pose3d getCombinedRobotPose(SwerveDrivePoseEstimator inputEstimationFromOdometry, Pose3d cameraEstimatedPose, double timestamp) {
        inputEstimationFromOdometry.addVisionMeasurement(cameraEstimatedPose.toPose2d(), timestamp);
    }

    private Pose3d getCombinedRobotPose(SwerveDrivePoseEstimator inputEstimationFromOdometry, Pose3d cameraOneEstimatedPosition, Pose3d cameraTwoEstimatedPosition) {
        inputEstimationFromOdometry.addVisionMeasurement(cameraOneEstimatedPosition.toPose2d(), cameraOneTimestamp);
        inputEstimationFromOdometry.addVisionMeasurement(cameraTwoEstimatedPosition.toPose2d(), cameraTwoTimestamp);
    }

    private void update(SwerveDrivePoseEstimator inputEstimationFromOdometry){
        boolean canTrustCameraOne;
        boolean canTrustCameraTwo;

        try {
            cameraOneEstimatedPosition = getEstimatedRobotPose(inputEstimationFromOdometry, CAMERA_ONE, HardwareConstants.ComponentTransformations._cameraOnePosition);
            cameraOneTimestamp = Timer.getFPGATimestamp();
            canTrustCameraOne = true;
        } catch (NullPointerException exception) {
            canTrustCameraOne = false;
        }

        try {
            cameraTwoEstimatedPosition = getEstimatedRobotPose(inputEstimationFromOdometry, CAMERA_TWO, HardwareConstants.ComponentTransformations._cameraTwoPosition);
            cameraTwoTimestamp = Timer.getFPGATimestamp();
            canTrustCameraTwo = true;
        } catch (NullPointerException exception) {
            canTrustCameraTwo = false;
        }

        if (canTrustCameraOne && canTrustCameraTwo) {
            combinedEstimatedPosition = getCombinedRobotPose(inputEstimationFromOdometry, cameraOneEstimatedPosition, cameraTwoEstimatedPosition).toPose2d();
        } else if (canTrustCameraOne) {
            combinedEstimatedPosition = getCombinedRobotPose(inputEstimationFromOdometry, cameraOneEstimatedPosition, cameraOneTimestamp).toPose2d();
        } else if (canTrustCameraTwo) {
            combinedEstimatedPosition = getCombinedRobotPose(inputEstimationFromOdometry, cameraTwoEstimatedPosition, cameraTwoTimestamp).toPose2d();
        } else {
            // Do nothing because we can't trust the cameras :(. 
        }
    }
}   

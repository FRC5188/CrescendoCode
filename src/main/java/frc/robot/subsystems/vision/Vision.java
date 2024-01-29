package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.vision.VisionIO;
import frc.robot.hardware.vision.VisionIOInputsAutoLogged;

public class Vision extends SubsystemBase {
  private final VisionIO _visionIO;
  private final VisionIOInputsAutoLogged _visionInputs = new VisionIOInputsAutoLogged();

  public Vision(VisionIO visionIO) {
    this._visionIO = visionIO;
  }

  public SwerveDrivePoseEstimator getVisionAccountedPoseEstimator(SwerveDrivePoseEstimator poseEstimatorFromOdometry) throws NullPointerException {
    try {
    _visionIO.setOdometryReferenceEstimator(poseEstimatorFromOdometry);
    poseEstimatorFromOdometry.addVisionMeasurement(
        new Pose2d(_visionInputs._cameraOneX, _visionInputs._cameraOneY, new Rotation2d(_visionInputs._cameraOneRotationRadians)),
        _visionInputs._cameraOneTimestamp
    );
    poseEstimatorFromOdometry.addVisionMeasurement(
        new Pose2d(_visionInputs._cameraTwoX, _visionInputs._cameraTwoY, new Rotation2d(_visionInputs._cameraTwoRotationRadians)),
        _visionInputs._cameraTwoTimestamp
    );

    return poseEstimatorFromOdometry;
    } catch (Exception exception) {
      throw new NullPointerException("The likley issue is that we're not looking at an Apriltag.");
    }
  }

  // Note: These timestamp functions should be run after pose estimator has been set.
  public double getCameraOneTimestamp() {
    return _visionInputs._cameraOneTimestamp;
  }

  public double getCameraTwoTimestamp() {
    return _visionInputs._cameraTwoTimestamp;
  }

  public void periodic() {
    _visionIO.updateInputs(_visionInputs);
    Logger.processInputs("Vision", _visionInputs);
  }
}

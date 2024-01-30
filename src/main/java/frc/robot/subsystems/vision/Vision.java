package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.vision.VisionIO;
import frc.robot.hardware.vision.VisionIOInputsAutoLogged;

public class Vision extends SubsystemBase {
  private final VisionIO _visionIO;
  private final VisionIOInputsAutoLogged _visionInputs = new VisionIOInputsAutoLogged();

  public Vision(VisionIO visionIO) {
    this._visionIO = visionIO;
  }

  @AutoLogOutput 
  public Pose2d getEstimatedPoseFrom(SwerveDrivePoseEstimator poseEstimatorFromOdometry) throws RuntimeException {
    this._visionIO.setOdometryReferenceEstimator(poseEstimatorFromOdometry);
    return _visionInputs._combinedPose;
  }

  public void periodic() {
    _visionIO.updateInputs(_visionInputs);
    Logger.processInputs("Vision", _visionInputs);
  }
}

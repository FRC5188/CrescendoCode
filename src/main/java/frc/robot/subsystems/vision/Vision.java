package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.vision.VisionIO;
import frc.robot.hardware.vision.VisionIOInputsAutoLogged;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.commands.CmdDefaultVision;

public class Vision extends SubsystemBase {
  private final VisionIO _visionIO;
  private final VisionIOInputsAutoLogged _visionInputs = new VisionIOInputsAutoLogged();
  private Drive _driveSubsystem;

  public Vision(VisionIO visionIO, Drive driveSubsystem) {
    this._visionIO = visionIO;
    this._driveSubsystem = driveSubsystem;

    this.setDefaultCommand(
      new CmdDefaultVision(this, this._driveSubsystem)
    );
  }

  public Pose3d[] getPoseList() {
    return _visionInputs._poseList;
  }

  public double[] getTimestampList() {
    return _visionInputs._timestampList;
  }

  public void periodic() {
    _visionIO.updateInputs(_visionInputs);
    Logger.processInputs("Vision", _visionInputs);
  }
}

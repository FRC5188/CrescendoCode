// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.visiondrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionDrive extends SubsystemBase {

  // final double CAMERA_HEIGHT_IN_METERS = 0.635;
  // final double CAMERA_PITCH_RADIANS = 20.0 / 180 * (3.14159);
  // final double GOAL_RANGE_METERS = 0.1;

  private final VisionDriveIO _visionDriveIO;

  private final VisionDriveIOInputsAutoLogged _visionDriveInputs = new VisionDriveIOInputsAutoLogged();

  private ChassisSpeeds _chassisSpeeds;
  private PIDController _translatePID;
  private PIDController _rotatePID;

  // TODO: Do I need to pass in VisionDriveIO? It's all just inputs.
  public VisionDrive(VisionDriveIO visionDriveIO) {
    _visionDriveIO = visionDriveIO;
    _chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  // TODO: What do I do with these??? :(
    _translatePID = new PIDController(0.2, 0, 0);
    _rotatePID = new PIDController(0.1, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _visionDriveIO.updateInputs(_visionDriveInputs);
  }

  public boolean hasTarget() {
    return 1.0 == _visionDriveInputs._seesNote;
  }

  public ChassisSpeeds getChassisSpeedsForDriveToNote() {
        if (hasTarget()) {
            
            // Note: x and y on the Limelight screen resemble the coordinate plane, with x increasing to the right and y increasing up.
            // x and y in ChassisSpeeds are different, with negative x representing forward translation and positive y representing right rotation.
            _chassisSpeeds.vxMetersPerSecond = -1.0*_translatePID.calculate(_visionDriveInputs._ty, 0);
            _chassisSpeeds.omegaRadiansPerSecond = _rotatePID.calculate(_visionDriveInputs._tx, 0);

        } else {
            // Else: stop.
            _chassisSpeeds.vxMetersPerSecond = 0.0;
            _chassisSpeeds.vyMetersPerSecond = 0.0;
            _chassisSpeeds.omegaRadiansPerSecond = 0.0;
        }

        return _chassisSpeeds;
    }

    public ChassisSpeeds getChassisSpeedsForRotateAboutNote() {
      if (hasTarget()) {
          // Note: x and y on the Limelight screen resemble the coordinate plane, with x increasing to the right and y increasing up.
          // x and y in ChassisSpeeds are different, with negative x representing forward translation and positive y representing right rotation.
          _chassisSpeeds.omegaRadiansPerSecond = _rotatePID.calculate(_visionDriveInputs._tx, 0);

      } else {
          // Else: stop.
          _chassisSpeeds.omegaRadiansPerSecond = 0.0;
      }

      return _chassisSpeeds;
  }
}

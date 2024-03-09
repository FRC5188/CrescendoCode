// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.subsystems.visiondrive.VisionDriveIO;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private final GyroIO _gyroIO;
  private final GyroIOInputsAutoLogged _gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] _modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine _sysId;

  private final VisionIO _visionIO;
  public final VisionDriveIO _visionDriveIO;

  private final VisionIOInputsAutoLogged _visionInputs = new VisionIOInputsAutoLogged();

  private SwerveDriveKinematics _kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d _rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] _lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator _poseEstimator = new SwerveDrivePoseEstimator(_kinematics, _rawGyroRotation,
      _lastModulePositions, new Pose2d());

  private Alliance _alliance;
  private Pose2d _speakerPosition;
  private Translation2d _centerOfRotation;
  public Field2d _field;

  public Drive(
      GyroIO gyroIO,
      VisionIO visionIO,
      VisionDriveIO visionDriveIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this._gyroIO = gyroIO;
    this._visionIO = visionIO;
    this._visionDriveIO = visionDriveIO;

    _modules[0] = new Module(flModuleIO, 0);
    _modules[1] = new Module(frModuleIO, 1);
    _modules[2] = new Module(blModuleIO, 2);
    _modules[3] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> _kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            DriveConstants.MAX_LINEAR_SPEED, DriveConstants.DRIVE_BASE_RADIUS, new ReplanningConfig()),
        () -> DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    _sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            DriveConstants.SYSID_RAMP_RRATE,
            DriveConstants.SYSID_STEP_VOLTAGE,
            DriveConstants.SYSID_TIMEOUT,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              for (int i = 0; i < 4; i++) {
                _modules[i].runCharacterization(voltage.in(Volts));
              }
            },
            null,
            this));
    _centerOfRotation = new Translation2d();
    _field = new Field2d();
  }

  public void periodic() {
    _gyroIO.updateInputs(_gyroInputs);
    Logger.processInputs("Drive/Gyro", _gyroInputs);

    _visionIO.updateInputs(_visionInputs);
    Logger.processInputs("Drive/Vision", _visionInputs);

    for (var module : _modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : _modules) {
        module.stop();
      }

      // Log empty setpoint states when disabled
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] = new SwerveModulePosition(
          modulePositions[moduleIndex].distanceMeters
              - _lastModulePositions[moduleIndex].distanceMeters,
          modulePositions[moduleIndex].angle);
      _lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (_gyroInputs._connected) {
      // Use the real gyro angle
      _rawGyroRotation = _gyroInputs._yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = _kinematics.toTwist2d(moduleDeltas);
      _rawGyroRotation = _rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }
    // Apply odometry update
    for (int i = 0; i < HardwareConstants.NUMBER_OF_CAMERAS; i++) {
      if (_visionInputs._hasPose[i]) {
        _poseEstimator.addVisionMeasurement(_visionInputs._poses[i], _visionInputs._timestamps[i]);
      }
    }
    _poseEstimator.update(_rawGyroRotation, modulePositions);

    // MITCHELL READ THIS COMMENT. I'm still getting the loop overrun issue from the
    // driver station
    // so this did not solve the issue. but i wanted to bring it to your attention.
    // as of the commit which added this block of comments, the only errors I'm
    // getting from the driver
    // station is about photon vision cameras and the loop over run from drive
    // periodic. i am a little
    // worried that this drive periodic overrun issue might come back to bite us so
    // I don't want to
    // leave it not addressed for too long. But, first, lets get the rest of the
    // robot back up and going

    // commenting this out to see if it helps with loop overrun time gh - 2/25/24
    // driver.periodic is sometimes running at 20-40ms on its own
    // _field.setRobotPose(_poseEstimator.getEstimatedPosition());
    // SmartDashboard.putData("Field", _field);
    // double[] cor = {_centerOfRotation.getX(), _centerOfRotation.getY()};
    // SmartDashboard.putNumberArray("CoR", cor);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = _kinematics.toSwerveModuleStates(discreteSpeeds, _centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = _modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement.
   * The modules will
   * return to their normal orientations the next time a nonzero velocity is
   * requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    _kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return _sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return _sysId.dynamic(direction);
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = _modules[i].getState();
    }
    return states;
  }

  /**
   * Returns the module positions (turn angles and drive velocities) for all of
   * the modules.
   */
  // TODO: Garrett 2/27/24
  // should this return _module[i].getModueState() instead? Advantagekit cannot
  // show a SwerveModulePosition.
  // only a swerve module state.
  @AutoLogOutput(key = "SwerveStates/MeasuredPosition")
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = _modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return _poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    _poseEstimator.resetPosition(_rawGyroRotation, getModulePositions(), pose);
  }

  public Rotation2d getGyroscopeRotation() {
    return _rawGyroRotation;
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp  The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    _poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  private Alliance getAlliance() {
    if (_alliance == null) {
      if (DriverStation.getAlliance().isPresent()) {
        _alliance = DriverStation.getAlliance().get();
      }
    }

    return _alliance;
  }

  private Pose2d getSpeakerPos() {
    if (_speakerPosition == null) {
      if (getAlliance() != null) {
        _speakerPosition = (getAlliance() == DriverStation.Alliance.Blue) ? DriveConstants.BLUE_SPEAKER
            : DriveConstants.RED_SPEAKER;
      }
    }

    return _speakerPosition;
  }

  /**
   * Returns the distance from the center of the robot to the alliance's speaker
   */
  public double getRadiusToSpeakerInMeters() {

    return getRadiusToSpeakerInMeters(_poseEstimator.getEstimatedPosition(), getSpeakerPos());
  }
  
  /**
   * Returns the distance from the center of the robot to the alliance's speaker.
   * NOTE: Use THIS constructor for GrpShootOnTheMove only.
   */
  public double getRadiusToSpeakerInMeters(Pose2d robotPose) {

    return getRadiusToSpeakerInMeters(robotPose, getSpeakerPos());
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return _poseEstimator;
  }

  public ChassisSpeeds getRobotChassisSpeeds() {
    return _kinematics.toChassisSpeeds(this.getModuleStates());
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(this.getRobotChassisSpeeds(), this.getRotation());
  }

  public Drive getObject() {
    return this;
  }

  // this setup lets us test the math, but when we actually run the code we don't
  // have to give a pose estimator
  public static double getRadiusToSpeakerInMeters(Pose2d robotPose, Pose2d speakerPos) {
    double xDiff = robotPose.getX() - speakerPos.getX();
    double yDiff = robotPose.getY() - speakerPos.getY();
    double xPow = Math.pow(xDiff, 2);
    double yPow = Math.pow(yDiff, 2);
    // Use pythagorean thm to find hypotenuse, which is our radius
    return Math.sqrt(xPow + yPow);
  }

  public void setCenterOfRotationToSpeaker() {
    if (getAlliance() == Alliance.Blue) {
      setCenterOfRotation(calcSpeakerCoRForBlue(_poseEstimator.getEstimatedPosition(), getSpeakerPos()));
    } else {
      setCenterOfRotation(calcSpeakerCoRForRed(_poseEstimator.getEstimatedPosition(), getSpeakerPos()));
    }
  }

  public double calcAngleToSpeaker() {
    Pose2d robotPose = _poseEstimator.getEstimatedPosition();
    Pose2d speakerPos = getSpeakerPos();
    double xDiff = speakerPos.getX() - robotPose.getX();
    double yDiff = speakerPos.getY() - robotPose.getY();
    return Math.toDegrees(Math.atan(yDiff / xDiff));
  }

  /**
   * Thank you to Team 3467 - Windham Windup!
   * Adopted from 3467's getAngleToSpeaker:
   * https://github.com/WHS-FRC-3467/Skip-5.14-Nocturne/blob/PostGSDCleanup/src/main/java/frc/robot/Util/FieldCentricAiming.java
   * NOTE: Only used for GrpShootDriveOnTheMove; otherwise, use
   * calcAngleToSpeaker.
   */

  public Rotation2d getRotation2dToSpeaker(Translation2d futurePose) {
    return getSpeakerPos().getTranslation().minus(futurePose).getAngle();
  }

  public static Translation2d calcSpeakerCoRForBlue(Pose2d robotPose, Pose2d speakerPos) {
    double xDiff = robotPose.getX() - speakerPos.getX();
    double yDiff = speakerPos.getY() - robotPose.getY();
    return new Translation2d(xDiff, yDiff);
  }

  public static Translation2d calcSpeakerCoRForRed(Pose2d robotPose, Pose2d speakerPos) {
    double xDiff = speakerPos.getX() - robotPose.getX();
    double yDiff = speakerPos.getY() - robotPose.getY();
    return new Translation2d(xDiff, yDiff);
  }

  public void setCenterOfRotationToRobot() {
    setCenterOfRotation(new Translation2d());
  }

  private void setCenterOfRotation(Translation2d pos) {
    _centerOfRotation = pos;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return DriveConstants.MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
        new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0),
        new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
        new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0)
    };
  }
}

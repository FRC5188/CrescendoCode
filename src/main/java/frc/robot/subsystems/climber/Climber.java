package frc.robot.subsystems.climber;

import java.security.InvalidParameterException;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.climber.ClimberIO;

public class Climber extends SubsystemBase {
  private ClimberIO _hardware;
  private double _currentHeight;
  
  public Climber(ClimberIO hardware, double initialHeight) {
    _hardware = hardware;
    this._currentHeight = initialHeight;
  }

  @AutoLogOutput
  public double getCurrentHeight() {
    return this._currentHeight;
  }

  public void setClimberSpeed(double speed) {
    setClimberLeftSpeed(speed);
    setClimberRightSpeed(speed);
  }

  public void setClimberPosition(double heightFromRobotBaseMeters) throws Exception {
      if (heightFromRobotBaseMeters > ClimberConstants.MAXIMUM_HEIGHT_OF_CLIMBERS || heightFromRobotBaseMeters < 0) {
        throw new InvalidParameterException("Cannot Have A Height Outside of Domain of Climber.");
      }
      // Now that we've establised that our parameters work then we can do work. 
      this._currentHeight = ClimberConstants.DELTA_METERS_INTO_ROTATIONS * heightFromRobotBaseMeters;
      _hardware.setLeftClimberPosition(this._currentHeight);
      _hardware.setRightClimberPosition(this._currentHeight);
  }

  public void setClimberLeftSpeed(double speed) {
    _hardware.setLeftClimberVelocity(speed);
  }

  public void setClimberRightSpeed(double speed) {
    _hardware.setRightClimberVelocity(speed);
  }

  @Override
  public void periodic() {
  }
}

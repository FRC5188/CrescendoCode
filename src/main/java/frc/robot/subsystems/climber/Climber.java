package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.climber.ClimberIO;

public class Climber extends SubsystemBase {
  private ClimberIO _hardware;
  private boolean _canMove;
  public Climber(ClimberIO hardware) {
    _hardware = hardware;
    _canMove = false;
  }

  public boolean getCanMove() {
    return _canMove;
  }

  public void setClimberSpeed(double speed) {
    //setClimberLeftSpeed(speed);
    //setClimberRightSpeed(speed);
  }

  //public void setClimberLeftSpeed(double speed) {
  //  _hardware.getLeftClimberMotor().set(speed);
  //}

  //public void setClimberRightSpeed(double speed) {
  //  _hardware.getRightClimberMotor().set(speed);
  //}

  @Override
  public void periodic() {
  }
}


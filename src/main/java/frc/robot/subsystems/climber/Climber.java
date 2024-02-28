package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIOInputsAutoLogged;


public class Climber extends SubsystemBase {
  private ClimberIO _hardware;
  private boolean _canMove;
  private final ClimberIOInputsAutoLogged _climberInputs = new ClimberIOInputsAutoLogged();
  public Climber(ClimberIO hardware) {
    _hardware = hardware;
    _canMove = false;
  }

  public boolean getCanMove() {
    return _canMove;
  }

  public void setCanMove(boolean canMove) {
    _canMove = canMove;
  }

  public void setClimberSpeed(double speed) {
    setClimberLeftSpeed(speed);
    setClimberRightSpeed(speed);
  }

  public void setClimberLeftSpeed(double speed) {
    _hardware.setLeftClimberSpeed(speed);
  }

  public void setClimberRightSpeed(double speed) {
    _hardware.setRightClimberSpeed(speed);
  }

  public boolean isClimbing() {
    double leftSpeed = _climberInputs._leftClimberSpeed;
    double rightSpeed = _climberInputs._rightClimberSpeed;
    return (leftSpeed != 0 || rightSpeed != 0);
  }

  @Override
  public void periodic() {
  }
}

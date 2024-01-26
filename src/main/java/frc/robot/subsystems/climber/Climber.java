package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.climber.ClimberHardware;

public class Climber extends SubsystemBase {
  private ClimberHardware _hardware;
  private boolean _canMove;
  public Climber(ClimberHardware hardware) {
    _hardware = hardware;
    _canMove = false;
  }

  public boolean getCanMove() {
    return _canMove;
  }

  @Override
  public void periodic() {
  }
}

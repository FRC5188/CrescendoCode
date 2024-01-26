package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.shooter.ShooterHardware;

public class Shooter extends SubsystemBase {
  ShooterHardware _hardware;

  public Shooter(ShooterHardware hardware) {
    _hardware = hardware;
  }
  public CANSparkFlex configAngleMotor(CANSparkFlex angleMotor){
    return angleMotor;
  }

  @Override
  public void periodic() {
  }
}

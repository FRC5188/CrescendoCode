package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX2 implements GyroIO {
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  public GyroIONavX2() {
    gyro.zeroYaw();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    if (!gyro.isCalibrating()) {
      inputs.yawPosition = Rotation2d.fromDegrees(-gyro.getYaw());
      inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getRate());
    }
  }
}

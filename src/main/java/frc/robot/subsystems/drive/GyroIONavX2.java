package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX2 implements GyroIO {
  private final AHRS _gyro = new AHRS(SPI.Port.kMXP);

  public GyroIONavX2() {
    _gyro.zeroYaw();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs._connected = _gyro.isConnected();
    if (!_gyro.isCalibrating()) {
      inputs._yawPosition = Rotation2d.fromDegrees(-_gyro.getYaw());
      inputs._yawVelocityRadPerSec = Units.degreesToRadians(_gyro.getRate());
    }
  }
}

package frc.robot.subsystems.visiondrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface VisionDriveIO {
    
    public default ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds();
    }

    public default boolean hasTarget() {
        return false;
    }
}

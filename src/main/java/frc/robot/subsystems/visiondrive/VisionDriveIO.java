package frc.robot.subsystems.visiondrive;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.HardwareConstants;

public interface VisionDriveIO {
    
    @AutoLog
    public static class VisionDriveIOInputs {
        public double _setSpeed;
        public double _rot;
    }

    public default void updateInputs(VisionDriveIOInputs inputs) {}
}

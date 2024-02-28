package frc.robot.subsystems.visiondrive;

import org.littletonrobotics.junction.AutoLog;

public interface VisionDriveIO {
    
    @AutoLog
    public static class VisionDriveIOInputs {
        public double _forwardSpeed;
        public double _rotSpeed;
        public double _range;
    }

    public default void updateInputs(VisionDriveIOInputs inputs) {}

    public default boolean hasTarget() {
        return false;
    }
}

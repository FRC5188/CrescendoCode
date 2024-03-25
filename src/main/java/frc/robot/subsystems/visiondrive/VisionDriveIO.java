package frc.robot.subsystems.visiondrive;

import org.littletonrobotics.junction.AutoLog;

public interface VisionDriveIO {

    @AutoLog
    public static class VisionDriveIOInputs {
        public double _tx = 0.0;
        public double _ty = 0.0;
        public double _seesNote = 0.0;
    }

    public default void updateInputs(VisionDriveIOInputs inputs) {}
}

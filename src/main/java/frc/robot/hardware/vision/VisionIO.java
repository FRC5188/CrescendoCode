package frc.robot.hardware.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.hardware.HardwareConstants;

public interface VisionIO {
    
    @AutoLog
    public static class VisionIOInputs {
        // These are the estimated poses from each camera. Note that these indexes having meaning. 
        // If there is no value then we'll be given a null pose.  Null Pose: new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))
        public Pose3d[] _poseList = new Pose3d[HardwareConstants.NUMBER_OF_CAMERAS];
        public double[] _timestampList = new double[HardwareConstants.NUMBER_OF_CAMERAS];
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}

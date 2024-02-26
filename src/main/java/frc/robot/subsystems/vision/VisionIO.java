package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.HardwareConstants;

public interface VisionIO {
    
    @AutoLog
    public static class VisionIOInputs {
        // These are the estimated poses from each camera. Note that these indexes having meaning. 
        // If there is no value then we'll be given a null pose.  Null Pose: new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))
        public Pose2d[] _poses = new Pose2d[HardwareConstants.NUMBER_OF_PHOTONVISION_CAMERAS];
        public double[] _timestamps = new double[HardwareConstants.NUMBER_OF_PHOTONVISION_CAMERAS];
        public boolean[] _hasPose = new boolean[HardwareConstants.NUMBER_OF_PHOTONVISION_CAMERAS];
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}

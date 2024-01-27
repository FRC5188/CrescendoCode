package frc.robot.hardware.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public interface VisionIO {
    
    @AutoLog
    public static class VisionIOInputs {
        // We'll make this for two cameras right now though later we can add more. 
        public double _cameraOneX;
        public double _cameraOneY;
        public double _cameraOneTimestamp; 
        public double _cameraOneAmbiguity;
        public double _cameraOneTranslationTargetX;
        public double _cameraOneTranslationTargetY;

        public double _cameraTwoX;
        public double _cameraTwoY;
        public double _cameraTwoTimestamp;
        public double _cameraTwoAmbiguity;
        public double _cameraTwoTranslationTargetX;
        public double _cameraTwoTranslationTargetY;

        // While the variables above are for the cameras themselves the ones below are after they have been combined into one estimate.
        public double _combinedX;
        public double _combinedY;

        public double _translationToTargetX;
        public double _translationToTargetY;
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void setOdometryReferenceEstimator(SwerveDrivePoseEstimator inputEstimationFromOdometry) {} // This is passed in and then we use vision to account for error then the current estimated pose is put into the table.
}

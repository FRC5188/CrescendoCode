package frc.robot.subsystems.visiondrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface VisionDriveIO {
    
    /**
     * If a note is detected, get x and y of the note relative to the crosshair on the screen.
     * Calculate forward and rotation speeds using PIDs. Return a ChassisSpeeds object. 
     * If a note is not detected, the speeds will be set to 0.0.
     */
    public default ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds();
    }

    /**
     * Returns 1.0 if the Limelight recognizes a target. Returns 0.0 otherwise.
     */
    public default boolean hasTarget() {
        return false;
    }
}

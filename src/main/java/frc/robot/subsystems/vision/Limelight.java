package frc.robot.subsystems.vision;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Limelight extends SubsystemBase {

    private String _name;
    private NetworkTable _table;

    // Currently designed to pull the following values from the Limelight
    // See https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
    // From the Limelight NetworkTables API
    // botpose: double array: Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)

    private double[] _botpose = new double[7]; // Get this from the "botpose" entry in the tables
    private double _timestampSeconds;
    private boolean _isChanged;

    public Limelight(String name) {

        if (name.isBlank()) {
            this._name = ""; 
        } else {
            this._name = name;
        }
        // System.out.println("Starting Limelight");
        try {
            this._table = NetworkTableInstance.getDefault().getTable("limelight");
        } catch(Exception e) {
            // System.out.println("Limelight start failed with exception " + e);
          }
    }

    /**
     *  updateBotPose() parses the most recent Limelight NetworkTable message
     * and turns that into a wpilib Botpose
     */
    public void updatePose() {
        
        // Grab the most recent limelight botpose entry from Networktables
        double[] botpose7 = this._table.getEntry("botpose").getDoubleArray(new double[7]);

        // Should have 7 entries -- Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
        if(botpose7.length == 7) {  
            // The first 3 elements of this double array go to ~zero when it loses sight of an AprilTag. 
            // Sum them up and use that to determine if we see a tag.
            if ((botpose7[0] + botpose7[1] + botpose7[2]) < 0.01f) {
                this._isChanged = false;
            } else {
                this._isChanged = true;
                this._botpose = botpose7;
                // Pull the time and then subtact off the latency of the camera to generate a timestamp.
                this._timestampSeconds = Timer.getFPGATimestamp() - (this._botpose[6]/1000.0);
                // System.out.println("Timestamp " + this._timestampSeconds + ": Bot pose changed to " + Arrays.toString(this._botpose));
            }
        } // else { 
        //     // System.out.println("Bad botpose data received from getBotPose()");
        // }
    }

    /** getPose2d
     * Returns the most recent robot pose in a Pose2d format
     * @return: Pose2d
     */
    public Pose2d getPose2d() {
        Translation2d trans2d = new Translation2d(this._botpose[0], this._botpose[1]);
        Rotation2d rot2d = new Rotation2d(Units.degreesToRadians(this._botpose[5]));
        return new Pose2d(trans2d, rot2d);
    }

    /** getPose3d
     * Returns the most recent robot pose in a Pose3d format
     * @return: Pose3d
     */
    public Pose3d getPose3d() {
        Translation3d trans3d = new Translation3d(this._botpose[0], this._botpose[1], this._botpose[2]);
        Rotation3d rot3d = new Rotation3d(Units.degreesToRadians(this._botpose[3]), 
            Units.degreesToRadians(this._botpose[4]), Units.degreesToRadians(this._botpose[5]));
        return new Pose3d(trans3d, rot3d);
    }

    /** getTimeStampSeconds
     * Returns the timestamp (in seconds since the robot startup) of the most recent robot pose update
     * @return: double
     */
    public double getTimestampSeconds() {
        return this._timestampSeconds;
    }

    public boolean poseIsChanged() {
        return this._isChanged;
    }

    public double getPosX() {
        return this._botpose[0];
    }
    
    public double getPosY() {
        return this._botpose[1];
    }

    public double getPosZ() {
        return this._botpose[2];
    }

    public void setLEDMode(LEDMode mode) {

        switch (mode) {
            case PIPELINEDEFAULT:
                this._table.getEntry("ledMode").setNumber(0);
                break;
            case FORCE_OFF:
                this._table.getEntry("ledMode").setNumber(1);
                break;
            case FORCE_BLINK:
                this._table.getEntry("ledMode").setNumber(2);
                break;
            case FORCE_ON:
                this._table.getEntry("ledMode").setNumber(3);
                break;
        }
    }

    @Override
    public void periodic(){
        updatePose();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Limelight");
      builder.addDoubleProperty("positionX", this::getPosX, null);
      builder.addDoubleProperty("positionY", this::getPosY, null);
      builder.addDoubleProperty("positionZ", this::getPosZ, null);
    }

    public enum LEDMode {
        PIPELINEDEFAULT,
        FORCE_OFF,
        FORCE_BLINK,
        FORCE_ON
    }

    // public Pose2d getRelativeNotePose2d() {

    // }
}


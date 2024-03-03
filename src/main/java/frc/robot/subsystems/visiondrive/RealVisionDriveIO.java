package frc.robot.subsystems.visiondrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RealVisionDriveIO implements VisionDriveIO {

    final double CAMERA_HEIGHT_IN_METERS = 0.635;

    final double CAMERA_PITCH_RADIANS = 20.0 / 180 * (3.14159);

    final double GOAL_RANGE_METERS = 0.1;

    private PIDController _translatePID;
    private PIDController _rotatePID;

    private ChassisSpeeds _chassisSpeeds;

    private double _tx;
    private double _ty;

    private NetworkTable _table;

    public RealVisionDriveIO() {
        _translatePID = new PIDController(0.5, 0, 0);
        _rotatePID = new PIDController(7.0, 0, 0);

        _chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        _table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget() {
        return 1.0 == _table.getEntry("tv").getDouble(0.0);
    }

    // public void updateInputs(VisionDriveIOInputs inputs) {
        
    //     }

    public ChassisSpeeds getChassisSpeeds() {
        if (hasTarget()) {

            System.out.print("Target acquired! ");
            _tx = _table.getEntry("tx").getDouble(0.0);
            _ty = _table.getEntry("ty").getDouble(0.0);
            
            _chassisSpeeds.vxMetersPerSecond = _translatePID.calculate(_tx, 0);
            _chassisSpeeds.omegaRadiansPerSecond = _rotatePID.calculate(_ty, 0);

            return _chassisSpeeds;

        } else {
            _chassisSpeeds.vxMetersPerSecond = 0.0;
            _chassisSpeeds.vyMetersPerSecond = 0.0;
            _chassisSpeeds.omegaRadiansPerSecond = 0.0;
        }

        System.out.println(" vx is:" + _chassisSpeeds.vxMetersPerSecond + " and vy is: " + _chassisSpeeds.vyMetersPerSecond);

        return _chassisSpeeds;
    }
}

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
        _translatePID = new PIDController(0.1, 0, 0);
        _rotatePID = new PIDController(0.05, 0, 0);

        _chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        _table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget() {
        return 1.0 == _table.getEntry("tv").getDouble(0.0);
    }

    public ChassisSpeeds getChassisSpeeds() {
        if (hasTarget()) {
            _tx = _table.getEntry("tx").getDouble(0.0);
            _ty = _table.getEntry("ty").getDouble(0.0);
            
            // Note: x and y on the Limelight screen resemble the coordinate plane, with x increasing to the right and y increasing up.
            // x and y in ChassisSpeeds are different, with negative x representing forward translation and positive y representing right rotation.
            _chassisSpeeds.vxMetersPerSecond = -1.0*_translatePID.calculate(_ty, 0);
            _chassisSpeeds.omegaRadiansPerSecond = _rotatePID.calculate(_tx, 0);

        } else {
            // Else: stop.
            _chassisSpeeds.vxMetersPerSecond = 0.0;
            _chassisSpeeds.vyMetersPerSecond = 0.0;
            _chassisSpeeds.omegaRadiansPerSecond = 0.0;
        }

        return _chassisSpeeds;
    }
}

package frc.robot.subsystems.visiondrive;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RealVisionDriveIO implements VisionDriveIO {

    final double CAMERA_HEIGHT_IN_METERS = 0.635;

    final double CAMERA_PITCH_RADIANS = 20.0 / 180 * (3.14159);

    final double GOAL_RANGE_METERS = 0.1;

    private PIDController _translatePID;
    private PIDController _rotatePID;

    private double _tx;
    private double _ty;

    private NetworkTable _table;

    public RealVisionDriveIO() {
        _translatePID = new PIDController(0.1, 0, 0.05);
        _rotatePID = new PIDController(7.0, 0, 0.05);

        _table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget() {
        return 1.0 == _table.getEntry("tv").getDouble(0.0);
    }

    public void updateInputs(VisionDriveIOInputs inputs) {
        if (hasTarget()) {
            _tx = _table.getEntry("tx").getDouble(100.0);
            _ty = _table.getEntry("ty").getDouble(100.0);

            inputs._range = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_IN_METERS,
                    0.0,
                    CAMERA_PITCH_RADIANS,
                    _ty);
            
            inputs._forwardSpeed = -_translatePID.calculate(inputs._range, GOAL_RANGE_METERS);
            inputs._rotSpeed = -_rotatePID.calculate(_tx, 0);
        }
    }
}

package frc.robot.subsystems.visiondrive;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.multisubsystemcommands.CmdShootOnTheMove;

public class RealVisionDriveIO implements VisionDriveIO {
    private NetworkTable _table;

    public RealVisionDriveIO() {
        this._table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void updateInputs(VisionDriveIOInputs inputs) {
        inputs._tx = this._table.getEntry("tx").getDouble(0.0);
        inputs._ty = this._table.getEntry("ty").getDouble(0.0);
        inputs._seesNote = this._table.getEntry("tv").getDouble(0.0);
    }
}

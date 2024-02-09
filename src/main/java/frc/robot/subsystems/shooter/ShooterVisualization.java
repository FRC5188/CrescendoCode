package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterVisualization {
    private final Mechanism2d _shooterVisualizer;
    private final MechanismRoot2d _shooterRoot;
    private final MechanismLigament2d _shooter;


    // shooter dimensions: 
    public ShooterVisualization() {
        _shooterVisualizer = new Mechanism2d(ShooterConstants.SHOOTER_WIDTH, ShooterConstants.SHOOTER_HEIGHT);
        _shooterRoot =_shooterVisualizer.getRoot("shooterRoot", ShooterConstants.SHOOTER_ROOT_WIDTH, ShooterConstants.SHOOTER_ROOT_HEIGHT);
        _shooter = _shooterRoot.append(new MechanismLigament2d("shooter", ShooterConstants.SHOOTER_LENGTH, ShooterConstants.SHOOTER_ANGLE));
        SmartDashboard.putData("ShooterVisualizer", _shooterVisualizer);
    }

    public void update(double shooterAngle) {
        _shooter.setAngle(Units.radiansToDegrees(shooterAngle));
        
    }
}

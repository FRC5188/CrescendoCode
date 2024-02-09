

package frc.robot.subsystems.intake;


import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

/** Add your docs here. */
public class IntakeVisualizer {

    // the main mechanism object
    Mechanism2d _intakeVisualizer;
    // the mechanism root node
    // TODO: Update origin values.
    MechanismRoot2d root;
     

     public IntakeVisualizer() {
        _intakeVisualizer = new Mechanism2d(IntakeConstants.INTAKE_WIDTH, IntakeConstants.INTAKE_HEIGHT);
        root = _intakeVisualizer.getRoot("Intake", 2, 0);
     }
     public void update(double pivotAngle) {
    
     }
}

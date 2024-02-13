

package frc.robot.subsystems.intake;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class IntakeVisualizer {

  private static final Translation2d _rootPosition = new Translation2d(0.28, 0.197);
  private Mechanism2d _mechanism;
  private MechanismRoot2d _mechanismRoot;
  private MechanismLigament2d _mechanismLigament;
     
   public IntakeVisualizer() {
      // Create mechanism
      _mechanism = new Mechanism2d(IntakeConstants.INTAKE_WIDTH, IntakeConstants.INTAKE_HEIGHT, new Color8Bit(Color.kGray));
      _mechanismRoot = _mechanism.getRoot("Intake",  2.0 + _rootPosition.getX(), _rootPosition.getY());
      _mechanismLigament = _mechanismRoot.append(
            new MechanismLigament2d("IntakeArm", IntakeConstants.INTAKE_LENGTH, 90, 4, new Color8Bit(Color.kLightGreen))
            );
  
     }
     public void update(double pivotAngle) {
         _mechanismLigament.setAngle(pivotAngle);
     }

     public Mechanism2d getMechanism(){
      return _mechanism;
     }

     public Pose3d getPose3d(double angle){
         return new Pose3d(
            _rootPosition.getX(), 0.0, _rootPosition.getY(), new Rotation3d(0.0, -angle, 0.0));
     }
}

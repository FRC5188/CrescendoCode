// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class ShooterVisualizer {

  private static Translation2d _rootPosition = new Translation2d(0.28, 0.197);
  private Mechanism2d _mechanism;
  private MechanismRoot2d _mechanismRoot;
  private MechanismLigament2d _mechanismLigament;
     
   public ShooterVisualizer() {
      // Create mechanism
      _mechanism = new Mechanism2d(ShooterConstants.SHOOTER_WIDTH, 
                                    ShooterConstants.SHOOTER_HEIGHT, 
                                    new Color8Bit(Color.kGray));
      _mechanismRoot = _mechanism.getRoot("shooter",  2.0 + _rootPosition.getX(), _rootPosition.getY());
      _mechanismLigament = _mechanismRoot.append(
            new MechanismLigament2d("ShooterArm", 
            ShooterConstants.SHOOTER_LENGTH, 
            ShooterConstants.SHOOTER_OFFSET_DEGREES, 
            4, new Color8Bit(Color.kLightGreen))
        );

        // SmartDashboard.putData("Shooter_Mech", _mechanism);
     }
     
     /***
      * 
      * @param pivotAngle angle in degrees
      */
     public void update(double pivotAngleDegrees) {
         _mechanismLigament.setAngle(new Rotation2d(Math.toRadians(-pivotAngleDegrees + ShooterConstants.SHOOTER_OFFSET_DEGREES)));
     }

     public Mechanism2d getMechanism(){
      return _mechanism;
     }

     public Pose3d getPose3d(double angle){
         return new Pose3d(
            _rootPosition.getX(), 0.0, _rootPosition.getY(), new Rotation3d(0.0, -angle, 0.0));
     }
}

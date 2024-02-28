// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;


import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class ShooterVisualizer {

    private static Translation2d _rootPosition = new Translation2d(0.28, 0.197);
    @AutoLogOutput(key = "Mechanism2D/Shooter")
    private Mechanism2d _mechanism;
    private MechanismRoot2d _mechanismRoot;
    private MechanismLigament2d _fixedLigament;
    private MechanismLigament2d _pivotLigament;
     
    public ShooterVisualizer() {
      // Create mechanism
      _mechanism = new Mechanism2d(ShooterConstants.SHOOTER_CANVAS_WIDTH, 
                                    ShooterConstants.SHOOTER_CANVAS_HEIGHT, 
                                    new Color8Bit(Color.kGray));
      _mechanismRoot = _mechanism.getRoot("shooter",  2.0 + _rootPosition.getX(), _rootPosition.getY());
      _fixedLigament = _mechanismRoot.append(
            new MechanismLigament2d("ShooterBase", 
            ShooterConstants.SHOOTER_HEIGHT_FROM_BASE, 
            90, 
            6, new Color8Bit(Color.kDarkBlue)));
            _pivotLigament = _fixedLigament.append(
            new MechanismLigament2d("ShooterPivot",
            ShooterConstants.SHOOTER_LENGTH,
            ShooterConstants.SHOOTER_OFFSET_DEGREES,
            4, 
            new Color8Bit(Color.kAquamarine)));
     }
     
     /***
      * 
      * @param pivotAngle angle in degrees
      */
     public void update(double pivotAngleDegrees) {
         _pivotLigament.setAngle(new Rotation2d(Math.toRadians(-pivotAngleDegrees + ShooterConstants.SHOOTER_OFFSET_DEGREES)));
     }

     public Mechanism2d getMechanism(){
      return _mechanism;
     }

     public Pose3d getPose3d(double angle){
         return new Pose3d(
            _rootPosition.getX(), 0.0, _rootPosition.getY(), new Rotation3d(0.0, -angle, 0.0));
     }
}

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.hardware.intake.IntakeHardware;

public class Intake extends SubsystemBase {
    public enum IntakePosition {
        SourcePickup,
        GroundPickup,
        Stowed,
        AmpScore,
        SpeakerScore
    }

    protected IntakePosition _intakePosition;

    public IntakePosition getIntakePosition() {
        return null;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

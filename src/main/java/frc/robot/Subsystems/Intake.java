package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public enum IntPos {
        Ground, Source, Middle, High
    }
    protected IntPos _curPos;
    public IntPos getIntakePosition() {
            return this._curPos;
    }


}

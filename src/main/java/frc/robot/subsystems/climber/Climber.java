package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.climber.ClimberIO;

public class Climber extends SubsystemBase {
    private ClimberIO _hardware;
    private boolean _canMove;

    public Climber(ClimberIO hardware) {
        _hardware = hardware;
        _canMove = false;
    }

    public boolean getCanMove() {
        return _canMove;
    }

    public void setCanMove(boolean canMove) {
        _canMove = canMove;
    }

    public void setClimberSpeed(double speed) {
        setClimberLeftSpeed(speed);
        setClimberRightSpeed(speed);
    }

    public void setClimberLeftSpeed(double speed) {
        _hardware.setLeftClimberVelocity(speed);
    }

    public void setClimberRightSpeed(double speed) {
        _hardware.setRightClimberPosition(speed);
    }

    @Override
    public void periodic() {
    }
}

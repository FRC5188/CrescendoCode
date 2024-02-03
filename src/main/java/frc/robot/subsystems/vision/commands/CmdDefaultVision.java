package frc.robot.subsystems.vision.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class CmdDefaultVision extends Command {
    private Vision _visionSubsystem;
    private Drive _driveSubsystem;

    public CmdDefaultVision(Vision visionSubsystem, Drive driveSubsystem) {
        this._visionSubsystem = visionSubsystem;
        this._driveSubsystem = driveSubsystem;

        addRequirements(visionSubsystem, _driveSubsystem);
    }

    @Override
    public void execute() {
        _visionSubsystem.periodic();

        for (int i = 0; i < _visionSubsystem.getPoseList().length; i++) {
            _driveSubsystem.addVisionMeasurement(
                _visionSubsystem.getPoseList()[i].toPose2d(),
                _visionSubsystem.getTimestampList()[i]
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

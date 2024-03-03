package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class CmdShooterWaitUntilReady extends Command {
    private Shooter _shooterSubsystem;
    private Intake _intakeSubsystem;

    public CmdShooterWaitUntilReady(Shooter shooterSubsystem, Intake intakeSubsystem) {
        _shooterSubsystem = shooterSubsystem;
        _intakeSubsystem = intakeSubsystem;
        System.out.println("Wait until ready initialized");

        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        

    }

    @Override
    public boolean isFinished() {
        // for now, we only cared if the shooter was ready, not the intake
        // brady worked on this on saturday week 1

        //System.out.println(_shooterSubsystem.isReady());
        //System.out.println(_intakeSubsystem.pivotAtSetpoint());
        //System.out.println(_intakeSubsystem.getIntakePosition());
        return _shooterSubsystem.isReady();
        //return _shooterSubsystem.isReady() && _intakeSubsystem.pivotAtSetpoint() && _intakeSubsystem.getIntakePosition() == IntakePosition.Stowed;
    }
}

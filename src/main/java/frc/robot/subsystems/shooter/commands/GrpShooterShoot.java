// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.IntakeConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrpShooterShoot extends ParallelCommandGroup {
    Intake _intakeSubsystem;
    Shooter _shooterSubsystem;
    Drive _driveSubsystem;

    /** Creates a new GrpShooterShoot. */
    public GrpShooterShoot(Intake intakeSubsystem, Shooter shooterSubsystem, Drive driveSubsystem) {
        _intakeSubsystem = intakeSubsystem;
        _shooterSubsystem = shooterSubsystem;
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        if (true) {
            addCommands(new CmdShooterRunShooter(_shooterSubsystem, _intakeSubsystem, _driveSubsystem),
                    new CmdShooterWaitUntilReady(_shooterSubsystem,
                            _intakeSubsystem),
                    _intakeSubsystem.buildCommand().spit(IntakeConstants.INTAKE_SPIT_TIME),
                    new InstantCommand(
                            () -> {
                                this._shooterSubsystem.setFlywheelSpeed(0);
                            }, this._shooterSubsystem));
        }

    }
}

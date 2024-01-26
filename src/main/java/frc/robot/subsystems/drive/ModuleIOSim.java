// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim _driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
  private DCMotorSim _turnSim = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

  private final Rotation2d _turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double _driveAppliedVolts = 0.0;
  private double _turnAppliedVolts = 0.0;

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    _driveSim.update(LOOP_PERIOD_SECS);
    _turnSim.update(LOOP_PERIOD_SECS);

    inputs._drivePositionRad = _driveSim.getAngularPositionRad();
    inputs._driveVelocityRadPerSec = _driveSim.getAngularVelocityRadPerSec();
    inputs._driveAppliedVolts = _driveAppliedVolts;
    inputs._driveCurrentAmps = new double[] {Math.abs(_driveSim.getCurrentDrawAmps())};

    inputs._turnAbsolutePosition =
        new Rotation2d(_turnSim.getAngularPositionRad()).plus(_turnAbsoluteInitPosition);
    inputs._turnPosition = new Rotation2d(_turnSim.getAngularPositionRad());
    inputs._turnVelocityRadPerSec = _turnSim.getAngularVelocityRadPerSec();
    inputs._turnAppliedVolts = _turnAppliedVolts;
    inputs._turnCurrentAmps = new double[] {Math.abs(_turnSim.getCurrentDrawAmps())};
  }

  @Override
  public void setDriveVoltage(double volts) {
    _driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    _driveSim.setInputVoltage(_driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    _turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    _turnSim.setInputVoltage(_turnAppliedVolts);
  }
}

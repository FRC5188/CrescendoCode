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

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkFlex implements ModuleIO {
  // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final CANSparkFlex driveSparkFlex;
  private final CANSparkFlex turnSparkFlex;
  private final CANcoder cancoder;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final StatusSignal<Double> turnAbsolutePosition;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkFlex(int index) {
    switch (index) {
      case 0:
        driveSparkFlex = new CANSparkFlex(1, MotorType.kBrushless);
        turnSparkFlex = new CANSparkFlex(2, MotorType.kBrushless);
        cancoder = new CANcoder(3);
        absoluteEncoderOffset = Rotation2d.fromRotations(0.058630167643229); // MUST BE CALIBRATED
        break;
      case 1:
        driveSparkFlex = new CANSparkFlex(4, MotorType.kBrushless);
        turnSparkFlex = new CANSparkFlex(5, MotorType.kBrushless);
        cancoder = new CANcoder(6);
        absoluteEncoderOffset = Rotation2d.fromRotations(0.23297861735026); // MUST BE CALIBRATED
        break;
      case 2:
        driveSparkFlex = new CANSparkFlex(7, MotorType.kBrushless);
        turnSparkFlex = new CANSparkFlex(8, MotorType.kBrushless);
        cancoder = new CANcoder(9);
        absoluteEncoderOffset = Rotation2d.fromRotations(0.370921122233073); // MUST BE CALIBRATED
        break;
      case 3:
        driveSparkFlex = new CANSparkFlex(10, MotorType.kBrushless);
        turnSparkFlex = new CANSparkFlex(11, MotorType.kBrushless);
        cancoder = new CANcoder(12);
        absoluteEncoderOffset = Rotation2d.fromRotations(-0.360928141276042); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    cancoder.getConfigurator().apply(new CANcoderConfiguration());
    turnAbsolutePosition = cancoder.getAbsolutePosition();

    driveSparkFlex.restoreFactoryDefaults();
    turnSparkFlex.restoreFactoryDefaults();

    driveSparkFlex.setCANTimeout(250);
    turnSparkFlex.setCANTimeout(250);

    driveEncoder = driveSparkFlex.getEncoder();
    turnRelativeEncoder = turnSparkFlex.getEncoder();

    turnSparkFlex.setInverted(isTurnMotorInverted);
    driveSparkFlex.setSmartCurrentLimit(40);
    turnSparkFlex.setSmartCurrentLimit(30);
    driveSparkFlex.enableVoltageCompensation(12.0);
    turnSparkFlex.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkFlex.setCANTimeout(0);
    turnSparkFlex.setCANTimeout(0);

    driveSparkFlex.setPeriodicFramePeriod(
        CANSparkLowLevel.PeriodicFrame.kStatus3, 65520); // Analog Sensor Info
    driveSparkFlex.setPeriodicFramePeriod(
        CANSparkLowLevel.PeriodicFrame.kStatus4, 65522); // Alternate Encoder Info
    driveSparkFlex.setPeriodicFramePeriod(
        CANSparkLowLevel.PeriodicFrame.kStatus5, 65524); // Duty Cycle Encoder Position
    driveSparkFlex.setPeriodicFramePeriod(
        CANSparkLowLevel.PeriodicFrame.kStatus6, 65526); // Duty Cycle Encoder Velocity
    driveSparkFlex.setPeriodicFramePeriod(
        CANSparkLowLevel.PeriodicFrame.kStatus7, 65528); // Iaccum for PID

    turnSparkFlex.setPeriodicFramePeriod(
        CANSparkLowLevel.PeriodicFrame.kStatus3, 65520); // Analog Sensor Info
    turnSparkFlex.setPeriodicFramePeriod(
        CANSparkLowLevel.PeriodicFrame.kStatus4, 65522); // Alternate Encoder Info
    turnSparkFlex.setPeriodicFramePeriod(
        CANSparkLowLevel.PeriodicFrame.kStatus5, 65524); // Duty Cycle Encoder Position
    turnSparkFlex.setPeriodicFramePeriod(
        CANSparkLowLevel.PeriodicFrame.kStatus6, 65526); // Duty Cycle Encoder Velocity
    turnSparkFlex.setPeriodicFramePeriod(
        CANSparkLowLevel.PeriodicFrame.kStatus7, 65528); // Iaccum for PID

    driveSparkFlex.burnFlash();
    turnSparkFlex.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSparkFlex.getAppliedOutput() * driveSparkFlex.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkFlex.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkFlex.getAppliedOutput() * turnSparkFlex.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkFlex.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkFlex.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkFlex.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}

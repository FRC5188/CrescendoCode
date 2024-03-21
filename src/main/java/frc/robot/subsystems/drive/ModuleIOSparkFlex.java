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
import frc.robot.HardwareConstants;
import frc.robot.util.MotorFrameConfigurator;

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

  private final CANSparkFlex _driveSparkFlex;
  private final CANSparkFlex _turnSparkFlex;
  private final CANcoder _cancoder;

  private final RelativeEncoder _driveEncoder;
  private final RelativeEncoder _turnRelativeEncoder;
  private final StatusSignal<Double> _turnAbsolutePosition;

  private final boolean _isTurnMotorInverted = true;
  private final Rotation2d _absoluteEncoderOffset;

  public ModuleIOSparkFlex(int index) {
    switch (index) {
      case 0: //Front Left
        _driveSparkFlex = new CANSparkFlex(HardwareConstants.CanIds.FL_DRIVE, MotorType.kBrushless);
        _turnSparkFlex = new CANSparkFlex(HardwareConstants.CanIds.FL_TURN, MotorType.kBrushless);
        _cancoder = new CANcoder(HardwareConstants.CanIds.FL_CANCODER);
        _absoluteEncoderOffset = Rotation2d.fromRotations(DriveConstants.FL_OFFSET); // MUST BE CALIBRATED
        break;
      case 1: //Front Right
        _driveSparkFlex = new CANSparkFlex(HardwareConstants.CanIds.FR_DRIVE, MotorType.kBrushless);
        _turnSparkFlex = new CANSparkFlex(HardwareConstants.CanIds.FR_TURN, MotorType.kBrushless);
        _cancoder = new CANcoder(HardwareConstants.CanIds.FR_CANCODER);
        _absoluteEncoderOffset = Rotation2d.fromRotations(DriveConstants.FR_OFFSET); // MUST BE CALIBRATED
        break;
      case 2: //Back Left
        _driveSparkFlex = new CANSparkFlex(HardwareConstants.CanIds.BL_DRIVE, MotorType.kBrushless);
        _turnSparkFlex = new CANSparkFlex(HardwareConstants.CanIds.BL_TURN, MotorType.kBrushless);
        _cancoder = new CANcoder(HardwareConstants.CanIds.BL_CANCODER);
        _absoluteEncoderOffset = Rotation2d.fromRotations(DriveConstants.BL_OFFSET); // MUST BE CALIBRATED
        break;
      case 3: //Back Right
        _driveSparkFlex = new CANSparkFlex(HardwareConstants.CanIds.BR_DRIVE, MotorType.kBrushless);
        _turnSparkFlex = new CANSparkFlex(HardwareConstants.CanIds.BR_TURN, MotorType.kBrushless);
        _cancoder = new CANcoder(HardwareConstants.CanIds.BR_CANCODER);
        _absoluteEncoderOffset = Rotation2d.fromRotations(DriveConstants.BR_OFFSET); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    _cancoder.getConfigurator().apply(new CANcoderConfiguration());
    _turnAbsolutePosition = _cancoder.getAbsolutePosition();

    _driveSparkFlex.restoreFactoryDefaults();
    _turnSparkFlex.restoreFactoryDefaults();

    _driveSparkFlex.setCANTimeout(250);
    _turnSparkFlex.setCANTimeout(250);

    _driveEncoder = _driveSparkFlex.getEncoder();
    _turnRelativeEncoder = _turnSparkFlex.getEncoder();

    _turnSparkFlex.setInverted(_isTurnMotorInverted);
    _driveSparkFlex.setSmartCurrentLimit(40);
    _turnSparkFlex.setSmartCurrentLimit(30);
    _driveSparkFlex.enableVoltageCompensation(12.0);
    _turnSparkFlex.enableVoltageCompensation(12.0);
    _driveSparkFlex.setClosedLoopRampRate(0.25);
    _turnSparkFlex.setClosedLoopRampRate(0.25);

    _driveEncoder.setPosition(0.0);
    _driveEncoder.setMeasurementPeriod(10);
    _driveEncoder.setAverageDepth(2);

    _turnRelativeEncoder.setPosition(0.0);
    _turnRelativeEncoder.setMeasurementPeriod(10);
    _turnRelativeEncoder.setAverageDepth(2);

    MotorFrameConfigurator.configNoSensor(_driveSparkFlex);
    MotorFrameConfigurator.configNoSensor(_turnSparkFlex);
    
    _driveSparkFlex.setPeriodicFramePeriod(
        CANSparkLowLevel.PeriodicFrame.kStatus2, 10); // Motor position. 

    _driveSparkFlex.setCANTimeout(0);
    _turnSparkFlex.setCANTimeout(0);

    _driveSparkFlex.burnFlash();
    _turnSparkFlex.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs._drivePositionRad =
        Units.rotationsToRadians(_driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs._driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(_driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs._driveAppliedVolts = _driveSparkFlex.getAppliedOutput() * _driveSparkFlex.getBusVoltage();
    inputs._driveCurrentAmps = new double[] {_driveSparkFlex.getOutputCurrent()};
    inputs._driveRampRate = _driveSparkFlex.getClosedLoopRampRate();

    inputs._turnAbsolutePosition =
        Rotation2d.fromRotations(_turnAbsolutePosition.getValueAsDouble())
            .minus(_absoluteEncoderOffset);
    inputs._turnPosition =
      Rotation2d.fromRotations(_cancoder.getAbsolutePosition().getValueAsDouble() - _absoluteEncoderOffset.getRotations());
        //Rotation2d.fromRotations(_turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs._turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(_turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs._turnAppliedVolts = _turnSparkFlex.getAppliedOutput() * _turnSparkFlex.getBusVoltage();
    inputs._turnCurrentAmps = new double[] {_turnSparkFlex.getOutputCurrent()};
    inputs._turnRampRate = _turnSparkFlex.getClosedLoopRampRate();
  }

  @Override
  public void setDriveVoltage(double volts) {
    _driveSparkFlex.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    _turnSparkFlex.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    _driveSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    _turnSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setClosedLoopRampRate(double rampRate) {
    _driveSparkFlex.setClosedLoopRampRate(rampRate);
    _turnSparkFlex.setClosedLoopRampRate(rampRate);
  }
}

package frc.robot.util.feedfoward;

import edu.wpi.first.math.system.plant.DCMotor;

public abstract class NEOVortex {
  // FUNDEMENTIAL CONSTANTS FOR NEO VORTEX
  // Information from {@link https://docs.revrobotics.com/brushless/neo/compare}
  static final double NOMINAL_VOLTAGE_VOLTS = 12.0;
  static final double STALL_TORQUE_NEWTON_METERS = 3.6;
  static final double STALL_CURRENT_AMPS = 211.0;
  static final double FREE_CURRENT_AMPS = 3.62;
  static final double FREE_SPEED_RPM = 6784.0;

  /**
   * Returns a {@link DCMotor} implementation of a NEO Vortex. 
   * @param numMotors Number of NEO Vortex Motors Used.
   */
  public static DCMotor asMotor(int numMotors) {
    return new DCMotor(
      NOMINAL_VOLTAGE_VOLTS, 
      STALL_TORQUE_NEWTON_METERS, 
      STALL_CURRENT_AMPS, 
      FREE_CURRENT_AMPS, 
      FREE_SPEED_RPM,
      numMotors);
  }
}

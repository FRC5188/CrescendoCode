package frc.robot.util.feedfoward;

import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class SingleJointedArmFeedfoward {
  private static final boolean USES_ESTIMATED_VALUES = true;

  // EXPERIMENTIAL CONSTANTS (From SYSID Testing.)
  private static final double kS = 0.0;
  private static final double kG = 0.0;
  private static final double kV = 0.0;
  private static final double kA = 0.0;

  // THEORETICAL CONSTANTS
  private static final double ARM_MOMENT_OF_INERTIA = 0.0;
  private static final double ARM_GEARING = 0.0;
  private static final int NUMBER_OF_MOTORS = 1;
  private static final double MAXIMUM_ARM_SPEED_RADIAN = Units.degreesToRadians(45.0);
  private static final double MAXIMUM_ARM_ACCELERATION = Units.degreesToRadians(90.0);
  private static final double ESTIMATED_MODEL_ACCURACY_RADIANS = 0.015;
  private static final double ESTIMATED_MODEL_ACCURACY_RADIANS_PER_SECOND = 0.17;
  private static final double ESTIMATED_ENCODER_ACCURACY = 0.01;
  private static final double ERROR_TOLERANCE_POSITION_RADIANS = Units.degreesToRadians(1.0);
  private static final double ERROR_TOLERANCE_VELOCITY_RADIANS_PER_SECOND = Units.degreesToRadians(10.0);

  // PHYSICAL CONSTANTS 
  private static final double ROBOT_VOLTAGE = 12.0;

  // SOFTWARE CONSTANTS
  private static final double ROBOT_CYCLE_TIME_SECONDS = 0.020;

  private static final LinearSystem<N2, N1, N1> ARM =
      LinearSystemId.createSingleJointedArmSystem(
        NEOVortex.asMotor(NUMBER_OF_MOTORS),
        ARM_MOMENT_OF_INERTIA, 
        ARM_GEARING);

  private static final TrapezoidProfile PROFILE = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(
      MAXIMUM_ARM_SPEED_RADIAN, 
      MAXIMUM_ARM_ACCELERATION
    )
  );
        
  private static final KalmanFilter<N2, N1, N1> FILTER =
    new KalmanFilter<>(
      Nat.N2(), 
      Nat.N1(), 
      ARM, 
      VecBuilder.fill(
        ESTIMATED_MODEL_ACCURACY_RADIANS,
        ESTIMATED_MODEL_ACCURACY_RADIANS_PER_SECOND
      ), 
      VecBuilder.fill(ESTIMATED_ENCODER_ACCURACY), 
      ROBOT_CYCLE_TIME_SECONDS);

  private static final LinearQuadraticRegulator<N2, N1, N1> CONTROLLER = new LinearQuadraticRegulator<>(
    ARM,
    VecBuilder.fill(
      ERROR_TOLERANCE_POSITION_RADIANS,
      ERROR_TOLERANCE_VELOCITY_RADIANS_PER_SECOND
    ),
    VecBuilder.fill(ROBOT_VOLTAGE),
    ROBOT_CYCLE_TIME_SECONDS);
    
  private static final LinearSystemLoop<N2, N1, N1> LOOP = new LinearSystemLoop<>(
    ARM, 
    CONTROLLER,
    FILTER, 
    ROBOT_VOLTAGE,
    ROBOT_CYCLE_TIME_SECONDS
  );
  
  /**Used in holding the {@link TrapezoidProfile.State} of the robot arm.*/
  private TrapezoidProfile.State _state = new TrapezoidProfile.State();

  /**
   * Should be used whenever we want a feedfoward based on the values from SYSID. 
   */
  public SingleJointedArmFeedfoward() {
    if (USES_ESTIMATED_VALUES) throw new IllegalStateException("This constructor should not be used when not using SYSID.");
  }

  /**
   * Should be used whenever we want an an estimation of the feedfoward when not using SYSID. 
   * @param encoder
   */
  public SingleJointedArmFeedfoward(SparkAbsoluteEncoder encoder) {
    if (!USES_ESTIMATED_VALUES) throw new IllegalStateException("This constructor should not be used when using SYSID.");
    // Since the output of our encoder is in rotations we'll convert it into radians. 
    final double POSITION_RADIANS = encoder.getPosition() * (2 * Math.PI);
    final double VELOCITY_RADIANS_PER_SECOND = encoder.getVelocity() * (2 * Math.PI);

    this._state = new TrapezoidProfile.State(POSITION_RADIANS, VELOCITY_RADIANS_PER_SECOND);
  }

  /**
   * Calculates the feedfoward based on desired position in radians.
   * @param encoder Absolute Encoder on Arm. Depending on whether we're using SYSID or not, this may or may not be used.
   * @param angleRadians Desired Position in Radians.
   * @return Voltage that should be applied to arm.
   */
  public double calculate(SparkAbsoluteEncoder encoder, double angleRadians) {
    if (USES_ESTIMATED_VALUES) return calculateSYSID(angleRadians);
    return calculateEstimated(encoder, angleRadians);
  }


  /**
   * Uses SYSID values in calculating the feedfoward based on desied position in radians. 
   * @param angleRadians Desired Position of Arm in Radians
   * @return Estimated Voltage that Should Be Applied.
   */
  private double calculateSYSID(double angleRadians) {
      return new ArmFeedforward(kS, kG, kV, kA).calculate(angleRadians, 0.0);
  }

  /**
   * Uses the estimated value that are based on physical dimensions of the arm in calculating the feedfoward based on desied position in radians.
   * @param encoder Absolute Encoder Used on Arm.
   * @param angleRadians Desired Position of Arm in Radians.
   * @return Estimated Voltage that Should be Applied.
   */
  private double calculateEstimated(SparkAbsoluteEncoder encoder, double angleRadians) {

    final TrapezoidProfile.State target = new TrapezoidProfile.State(angleRadians, 0.0);
    final double position = encoder.getPosition() * (2 * Math.PI);

    this._state = PROFILE.calculate(
        ROBOT_CYCLE_TIME_SECONDS, 
        this._state,
        target);

    LOOP.setNextR(
        this._state.position,
        this._state.velocity
    );

    LOOP.correct(VecBuilder.fill(position));

    LOOP.predict(ROBOT_CYCLE_TIME_SECONDS);

    return LOOP.getU(0);
  }
}

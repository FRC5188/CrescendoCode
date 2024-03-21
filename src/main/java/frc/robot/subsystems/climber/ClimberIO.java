package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    public static class ClimberIOInputs {
        // |===================== LEFT CLIMBER MOTOR LOGGING =====================|
        public double _leftClimberMotorTemperature;
        public double _leftClimberMotorVelocityRotationsPerMin;
        public double _leftClimberMotorPositionRotations;
        public double _leftClimberMotorVoltage;
        public double _leftClimberMotorCurrent;
        public double _leftClimberSpeed;
        // |===================== END LEFT CLIMBER MOTOR LOGGING =====================|

        // |===================== RIGHT CLIMBER MOTOR LOGGING =====================|
        public double _rightClimberMotorTemperature;
        public double _rightClimberMotorVelocityRotationsPerMin;
        public double _rightClimberMotorPositionRotations;
        public double _rightClimberMotorVoltage;
        public double _rightClimberMotorCurrent;
        public double _rightClimberSpeed;
        // |===================== END RIGHT CLIMBER MOTOR LOGGING =====================|
    }

    public default void updateInputs(ClimberIOInputs inputs){}

    // |============================== LEFT CLIMBER MOTOR METHODS ============================== |
    public default void setLeftClimberSpeed(double speed){}

    // |============================== RIGHT CLIMBER MOTOR METHODS ============================== |
    public default void setRightClimberSpeed(double speed){}

    
}

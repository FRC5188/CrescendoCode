package frc.robot.hardware.climber;

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
        // |===================== END LEFT CLIMBER MOTOR LOGGING =====================|

        // |===================== RIGHT CLIMBER MOTOR LOGGING =====================|
        public double _rightClimberMotorTemperature;
        public double _rightClimberMotorVelocityRotationsPerMin;
        public double _rightClimberMotorPositionRotations;
        public double _rightClimberMotorVoltage;
        public double _rightClimberMotorCurrent;
        // |===================== END RIGHT CLIMBER MOTOR LOGGING =====================|
    }

    public default void updateInputs(ClimberIOInputs inputs){}

    // |============================== LEFT CLIMBER MOTOR METHODS ============================== |
    public default void setLeftClimberPosition(double positionRotations){}

    public default void setLeftClimberVoltage(double voltage){}

    public default void setLeftClimberVelocity(double velocityRotationsPerMin){}

    // |============================== RIGHT CLIMBER MOTOR METHODS ============================== |
    public default void setRightClimberPosition(double positionRotations){}

    public default void setRightClimberVoltage(double voltage){}

    public default void setRightClimberVelocity(double velocityRotationsPerMin){}
}

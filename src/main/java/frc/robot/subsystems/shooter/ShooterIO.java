package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
    @AutoLog
    public static class ShooterIOInputs {
        // |================= START LEFT FLYWHEEL MOTOR LOGGING =================|
        public double _leftFlywheelMotorTemperature;
        public double _leftFlywheelMotorVelocityRotationsPerMin;
        public double _leftFlywheelMotorVoltage;
        public double _leftFlywheelMotorCurrent;
        // |================= END LEFT FLYWHEEL MOTOR LOGGING =================|

        // |================= START RIGHT FLYWHEEL MOTOR LOGGING =================|
        public double _rightFlywheelMotorTemperature;
        public double _rightFlywheelMotorVelocityRotationsPerMin;
        public double _rightFlywheelMotorVoltage;
        public double _rightFlywheelMotorCurrent;
        // |================= END RIGHT FLYWHEEL MOTOR LOGGING =================|

        public double _flywheelPIDKP;
        public double _flywheelPIDKI;
        public double _flywheelPIDKD;
        public double _flywheelPIDISum;
        public double _flywheelPIDKF;
        
        // |================= START ANGLE MOTOR LOGGING =================|
        public double _angleMotorTemperature;
        public double _angleMotorPositionDegrees;
        public double _angleMotorVelocityRotationsPerMin;
        public double _angleMotorCurrent;
        public double _angleMotorVoltage;
        // |================= END ANGLE MOTOR LOGGING =================|

        // |================= START ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|
        public double _angleEncoderPositionDegrees;
        // |================= END ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|
    }

    public default void updateInputs(ShooterIOInputs inputs){}

    // |============================== FLYWHEEL MOTOR METHODS ============================== |

    public default void setLeftFlywheelMotorVelocity(double velocityRadiansPerSecond){}

    public default void setRightFlywheelMotorVelocity(double velocityRadiansPerSecond){}

    public default void stopFlywheels(){}

    public default void setFlywheelSpeedRPM(double velocityRotationsPerMinute) {}

    public default void setRightFlywheelSpeedRPM(double velocityRotationsPerMinute) {}

    // |============================== ANGLE MOTOR METHODS ============================== |

    public default void setTargetPositionAsDegrees(double degrees){}

    public default void setAngleMotorSpeed(double speed){}
}

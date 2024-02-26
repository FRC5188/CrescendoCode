package frc.robot.hardware.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        // |================= START PIVOT MOTOR LOGGING =================|
        public double _pivotMotorTemperature = 0.0; 
        public double _pivotMotorPositionDegrees = 0.0;
        public double _pivotMotorVelocityRotationsPerMin = 0.0;
        public double _pivotMotorCurrent = 0.0;
        public double _pivotMotorVoltage = 0.0;
        // i think this should be something we log?? GH 2/20/24 (pivotmotorpidsetpoint)
        // public double _pivotMotorPIDSetpoint;
        // |================= END PIVOT MOTOR LOGGING =================|

        // |================= START ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|
        public double _rollerMotorTemperature = 0.0;
        public double _rollerMotorVelocityRotationsPerMin = 0.0;
        public double _rollerMotorVoltage = 0.0;
        public double _rollerMotorCurrent = 0.0;
        // |================= END ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|

        // |================= START ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|
        public double _pivotEncoderPositionDegrees = 0.0;
        // |================= END ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    // |============================== PIVOT MOTOR METHODS ============================== |

    public default void setTargetPositionAsDegrees(double degrees) {}

    // |============================== ROLLER MOTOR METHODS ============================== |  
    public default void setRollerMotorSpeed(double speed) {}

    public default void setPivotMotorSpeed(double speed) {}
}

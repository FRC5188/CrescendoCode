package frc.robot.hardware.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        // |================= START PIVOT MOTOR LOGGING =================|
        public double _pivotMotorTemperature; 
        public double _pivotMotorPositionDegrees;
        public double _pivotMotorVelocityRotationsPerMin;
        public double _pivotMotorCurrent;
        public double _pivotMotorVoltage;
        // |================= END PIVOT MOTOR LOGGING =================|

        // |================= START ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|
        public double _rollerMotorTemperature;
        public double _rollerMotorVelocityRotationsPerMin;
        public double _rollerMotorVoltage;
        public double _rollerMotorCurrent;
        // |================= END ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|

        // |================= START ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|
        public double _pivotEncoderPositionDegrees;
        // |================= END ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    // |============================== PIVOT MOTOR METHODS ============================== |

    public default void setTargetPositionAsDegrees(double degrees) {}

    // |============================== ROLLER MOTOR METHODS ============================== |  
    public default void setRollerMotorSpeed(double speed) {}

    public default void setPivotMotorSpeed(double speed) {}
}

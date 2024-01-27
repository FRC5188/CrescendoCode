package frc.robot.hardware.intake;

import org.littletonrobotics.junction.AutoLog;

// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DigitalInput;

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
        public double _pivotEncoderPositionRotations;
        // |================= END ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|

        // |================= START LIGHT SENSOR LOGGING =================|
        public boolean _isLightSensorBlocked;
        // |================= END LIGHT SENSOR LOGGING =================|
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    // |============================== PIVOT MOTOR METHODS ============================== |
    public default void configPivotPID(double p, double i, double d) {}

    public default void setTargetPositionAsDegrees(double degrees) {}

    // |============================== ROLLER MOTOR METHODS ============================== |
    public default void setIsRollerRolling(boolean isRollerRolling) {}
}

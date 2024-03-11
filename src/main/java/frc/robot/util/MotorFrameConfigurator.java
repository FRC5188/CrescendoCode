package frc.robot.util;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

/**
 * This class is used to configure the periodic frame period of the CANSparkFlex motor controllers in efforts of reducing CAN bus traffic.
 */
public abstract class MotorFrameConfigurator {
    public static void configNoSensor(CANSparkFlex motor) {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100); // Applied output and faults. 
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Analog sensor info.
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // alternative encoder info.
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); // duty cycle encoder position
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); // duty cycle encoder velocity
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 65535); // Iaccum for PID
    }

    public static void configDutyCycleSensor(CANSparkFlex motor) {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100); // Applied output and faults. 
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Analog sensor info.
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // alternative encoder info.
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 65535); // Iaccum for PID
    }
}

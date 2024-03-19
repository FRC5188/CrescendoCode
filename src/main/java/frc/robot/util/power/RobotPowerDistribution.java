package frc.robot.util.power;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;
import frc.robot.util.power.battery.Battery;
import frc.robot.util.power.battery.BatteryStatus;

/// Abstract class used for power mangement of the various systems on the robot. 
public abstract class RobotPowerDistribution {
    /**CAN Bus Port for PDP. */
    private final static short PDP_PORT = 13;

    private final static double CLOSED_LOOP_RAMP_RATE_HALF_CHARGED = 0.1;
    private final static double CLOSED_LOOP_RAMP_RATE_LOW_CHARGE = 0.25;
    private final static double CLOSED_LOOP_RAMP_RATE_CRITICALLY_LOW_CHARGE = 1.0;

    private final static PowerDistribution PDP = new PowerDistribution(PDP_PORT, ModuleType.kCTRE);

    // Whether we want to give control of power management to the robot, or keep it in control of operator.
    private static boolean isAutomaticPowerManagementEnabled = false;
    private static BatteryStatus currentBatteryStatus = BatteryStatus.FULL_CHARGE;

    /**Give the computer control over ensuring we don't brown out. */
    public static void enableComputerManagement() {
        isAutomaticPowerManagementEnabled = true;
    }

    /**Give the operator control over changing motor configurations. */
    public static void disableComputerManagement() {
        isAutomaticPowerManagementEnabled = false;
    }

    /**Updates the battery status and the configuration based on the new status of the battery. */
    public static void update(Drive drive) {
        if (isAutomaticPowerManagementEnabled) {
            Battery.updateVoltage();
            configure(Battery.getStatus(), drive.getModules());
        } else {
            configure(currentBatteryStatus, drive.getModules());
        }
    }

    /**
     * Manually scheduled command that once started will continuously update the battery status and then configure the drive based on the new status.
     * @param drive Drive Subsystem.
     * @return Command updating battery status and configuring drive.
     */
    public static Command runCommand(Drive drive) {
        return new RunCommand(
            () -> {
                update(drive);
                updateLoggingInterfaces();
            }, drive
        );
    }

    /**
     * Will manually change the battery status and then disable automatic management of the battery.
     * @param status Status of Battery
     * @return Command changing battery status.
     */
    public static Command setBatteryStatusCommand(BatteryStatus status) {
        return Commands.runOnce(
            () -> setBatteryStatus(status));
    }

    /**When manually called will disable automatic management of the battery and then set current status to whatever is given. */
    public static void setBatteryStatus(BatteryStatus status) {
        // If we manually set the battery status, we want to disable automatic power management.
        disableComputerManagement();
        currentBatteryStatus = status;
    }

    private static void configure(BatteryStatus status, Module[] modules){
        switch (status) {
            case FULL_CHARGE:
                configureForFullCharge(modules);
                break;
            case HALF_CHARGE:
                configureForHalfCharge(modules);
                break;
            case LOW_CHARGE:
                configureForLowCharge(modules);
                break;
            case CRITICAL_CHARGE:
                configureForCriticalCharge(modules);
                break;
        }
    }

    private static void configureForFullCharge(Module[] modules) {
        // There is no configuration done at this point. The default configuration is as fast as we can go.
    }

    private static void configureForHalfCharge(Module[] modules) {
        for (Module module: modules){
            module.setClosedLoopRampRate(CLOSED_LOOP_RAMP_RATE_HALF_CHARGED);
        }
    }

    private static void configureForLowCharge(Module[] modules) {
        for (Module module: modules){
            module.setClosedLoopRampRate(CLOSED_LOOP_RAMP_RATE_LOW_CHARGE);
        }
    }

    private static void configureForCriticalCharge(Module[] modules) {
        for (Module module: modules){
            // Pretty signifigant reduction in acceleration as at this point our battery is in a bad state.
            module.setClosedLoopRampRate(CLOSED_LOOP_RAMP_RATE_CRITICALLY_LOW_CHARGE);
        }
    }

    private static double statusIntoClosedLoopRampRate(BatteryStatus status) {
        switch (status) {
            case FULL_CHARGE:
                return 0.0; // There is no change in driving.
            case HALF_CHARGE:
                return CLOSED_LOOP_RAMP_RATE_HALF_CHARGED;
            case LOW_CHARGE:
                return CLOSED_LOOP_RAMP_RATE_LOW_CHARGE;
            case CRITICAL_CHARGE:
                return CLOSED_LOOP_RAMP_RATE_CRITICALLY_LOW_CHARGE;
            default:
                return -100.0; // This should never happen.
        }
    }

    private static void updateLoggingInterfaces(){
        // We've updated Suffleboard and SmartDashboard to show the current battery status and the PDP voltage which should also be added into Advantage Scope.
        Shuffleboard.getTab("Power Distribution").add("Battery Status", currentBatteryStatus.toString());
        Shuffleboard.getTab("Power Distribution").add("PDP Voltage", PDP.getVoltage());
        Shuffleboard.getTab("Power Distribution").add("Programmed Acceleration Reduction", statusIntoClosedLoopRampRate(currentBatteryStatus));
    }
}

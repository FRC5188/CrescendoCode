package frc.robot.util.control.button;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public abstract class AbstractButtonFactory {
    /** The control that you have on the buttons. */
    public enum CONTROL {
        ON_TRUE_AND_ON_FALSE, // When PRESSING BUTTON and WHEN NOT
        WHILE_HOLDING_AND_WHILE_NOT, // When HOLDING BUTTON and WHEN NOT
        WHILE_ON_AND_WHILE_NOT, // When You've PRESSED THE BUTTON and When You Haven't
        ON_TRUE,
        ON_FALSE,
        WHILE_TRUE,
        WHILE_FALSE,
        WHEN_ON,
        WHEN_OFF
    }

    /**
     * @param controller Control Device Where Button Used (Xbox, Flight Joystick, etc.)
     * @param port Port of Button on Control Device
     * @param action The Action That's Preformed When Button Pressed. If An Action Requiring Multiple Then Array of Commands.
     * @return Configured Joystick Button
     */
    public static JoystickButton createButtonWithTrigger(GenericHID controller, int port, Command[] action, CONTROL control) {
        final JoystickButton BUTTON_BEING_CONFIGURED = new JoystickButton(controller, port);

        switch (control) {
            case ON_TRUE_AND_ON_FALSE:
                if (action.length != 2) throw new IllegalArgumentException("ON_TRUE_AND_ON_FALSE CONTROL REQUIRES TWO ACTIONS.");
                BUTTON_BEING_CONFIGURED.onTrue(action[0]).onFalse(action[1]);
                break;
            case WHILE_HOLDING_AND_WHILE_NOT:
                if (action.length != 2) throw new IllegalArgumentException("WHILE_HOLDING_AND_WHILE_NOT CONTROL REQUIRES TWO ACTIONS.");
                BUTTON_BEING_CONFIGURED.whileTrue(action[0]).whileFalse(action[1]);
                break;
            case WHILE_ON_AND_WHILE_NOT:
                if (action.length != 2) throw new IllegalArgumentException("WHILE_ON_AND_WHILE_NOT CONTROL REQUIRES TWO ACTIONS.");
                BUTTON_BEING_CONFIGURED.toggleOnTrue(action[0]).toggleOnFalse(action[1]);
                break;
            case ON_TRUE:
                BUTTON_BEING_CONFIGURED.onTrue(action[0]);
                break;
            case ON_FALSE:
                BUTTON_BEING_CONFIGURED.onFalse(action[0]);
                break;
            case WHILE_TRUE:
                BUTTON_BEING_CONFIGURED.whileTrue(action[0]);
                break;
            case WHILE_FALSE:
                BUTTON_BEING_CONFIGURED.whileFalse(action[0]);
                break;
            case WHEN_ON:
                BUTTON_BEING_CONFIGURED.toggleOnTrue(action[0]);
                break;
            case WHEN_OFF:
                BUTTON_BEING_CONFIGURED.toggleOnFalse(action[0]);
                break;
            default:
                throw new IllegalArgumentException("CONTROL TYPE SPECIFIED DOESN'T EXIST.");
        }
        
        return BUTTON_BEING_CONFIGURED;
    }

    /**
     * @param controller Control Device Where Button Used (Xbox, Flight Joystick, etc.)
     * @param port Port of Button on Control Device
     * @param action The Action That's Preformed When Button Pressed.
     * @return Configured Joystick Button
     */
    public static JoystickButton createButtonWithTrigger(GenericHID controller, int port, Command action, CONTROL control) {
        return createButtonWithTrigger(controller, port, new Command[]{action}, control);
    }
}

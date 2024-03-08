package frc.robot.util.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Adapter between the Joystick and XboxController classes for controlling the robot. */
public class DriveController {
    private XboxController _xbox;
    private Joystick _joystick;

    public static enum TYPE {
        JOYSTICK,
        XBOX
    }

    public DriveController(TYPE type, int port) {
        switch (type) {
            case JOYSTICK:
                _joystick = new Joystick(port);
                _xbox = null;
                break;
            case XBOX:
                _xbox = new XboxController(port);
                _joystick = null;
                break;
        }
    }

    public XboxController getXbox() throws NullPointerException{
        if (this._xbox == null) {
            throw new NullPointerException("Controller Not Configured for Xbox Control.");
        }
        return this._xbox;
    }

    public Joystick getJoystick() throws NullPointerException{
        if (this._joystick == null) {
            throw new NullPointerException("Controller Not Configured for Joystick Control.");
        }
        return this._joystick;
    }

    public JoystickButton[] getJoystickButtons(){
        return new JoystickButton[]{
            new JoystickButton(this._joystick, 1),
            new JoystickButton(this._joystick, 2),
            new JoystickButton(this._joystick, 3),
            new JoystickButton(this._joystick, 4),
            new JoystickButton(this._joystick, 5),
            new JoystickButton(this._joystick, 6),
            new JoystickButton(this._joystick, 7),
            new JoystickButton(this._joystick, 8),
            new JoystickButton(this._joystick, 9),
            new JoystickButton(this._joystick, 10),
            new JoystickButton(this._joystick, 11),
            new JoystickButton(this._joystick, 12)
        };
    }
}

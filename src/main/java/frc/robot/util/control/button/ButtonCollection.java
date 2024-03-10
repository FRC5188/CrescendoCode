package frc.robot.util.control.button;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public record ButtonCollection(Joystick joystick, JoystickButton[] buttons) {
    public JoystickButton getButton(int port) {
        return buttons[port];
    }
}

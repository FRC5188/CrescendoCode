package frc.robot.util.control.driving;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.control.DriveController.TYPE;

public class DriveHID {

    public enum DEVICE_TYPE {
        XBOX,
        FLIGHT_JOYSTICK
    }

    private GenericHID _device;
    private DEVICE_TYPE _type;

    public DriveHID(DEVICE_TYPE type, int port) {
        this._type = type;
        this._device = (this._type == DEVICE_TYPE.XBOX) ? 
            (configureXbox(port)) : 
            (configureFlightJoystick(port));
    }

    private GenericHID configureXbox(int port) {
        final XboxController xbox = new XboxController(port);

        // HERE IS WHERE WE'LL CON
    }

    private GenericHID configureFlightJoystick(int port) {

    }
}

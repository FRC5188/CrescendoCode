package frc.robot.util.control.driving;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.control.button.AbstractButtonFactory;
import frc.robot.util.control.button.ButtonCollection;

public class DriveHID {

    public enum DEVICE_TYPE {
        XBOX,
        FLIGHT_JOYSTICK
    }

    private Joystick _flightStick = null;
    private CommandXboxController _xbox = null;

    private Drive _drive;
    private DEVICE_TYPE _type;

    public DriveHID(DEVICE_TYPE type, int port, Drive drive) {
        this._type = type;
        this._drive = drive;
        if (this._type == DEVICE_TYPE.XBOX) {
            this._xbox = configureXbox(port);
        } else {
            this._flightStick = configureFlightJoystick(port);
        }
    }

    public CommandXboxController getXbox() {
        if (this._type == DEVICE_TYPE.XBOX) {
            return this._xbox;
        } else {
            throw new IllegalStateException("This DriveHID was not configured for an Xbox Controller.");
        }
    }

    public Joystick getFlightStick() {
        if (this._type == DEVICE_TYPE.FLIGHT_JOYSTICK) {
            return this._flightStick;
        } else {
            throw new IllegalStateException("This DriveHID was not configured for a Flight Joystick.");
        }
    }

    private CommandXboxController configureXbox(int port) {
        final CommandXboxController xbox = new CommandXboxController(port);

        // HERE IS WHERE WE'LL CONFIGURE THE BUTTONS FOR EACH YEAR
        // =================== XBOX MAPPINGS ===================
        // X - Stop Robot From Moving w/ X-Pattern
        // B - Rotate Robot to Face Speaker
        // Y - Reset Robot Orientation
        // A - Reset Robot Pose to be on Subwoofer

        // Makes an X-Shaped Pattern so that the Robot is hard to move.
        xbox.x().onTrue(_drive.buildCommand().stopWithX());

        // Face the Speaker.
        xbox.b().whileTrue(_drive.buildCommand().rotateAboutSpeaker(
            () -> -xbox.getLeftY(),
            () -> -xbox.getLeftX()));
        
        // Reset the orientation of the robot. changes which way it thinks is forward
        xbox.y().onTrue(_drive.buildCommand().resetRobotFront());

        // Reset the robot's pose to be on the subwoofer.
        xbox.a().onTrue(_drive.buildCommand().resetRobotPoseSpeaker());

        _drive.setDefaultCommand(_drive.buildCommand().drive(
            () -> -xbox.getLeftY(), 
            () -> -xbox.getLeftX(), 
            () -> -xbox.getRightX())
        );

        return xbox;
    }

    private Joystick configureFlightJoystick(int port) {
        // =================== FLIGHT JOYSTICK MAPPINGS ===================
        // Button 1 - Stop Robot From Moving w/ X-Pattern
        // Button 2 - Rotate Robot to Face Speaker
        // Button 3 - Reset Robot Orientation
        // Button 4 - Reset Robot Pose to be on Subwoofer

        final Joystick flightStick = new Joystick(port);
        final ButtonCollection buttons = new ButtonCollection( // CREATED THOUGH NEVER USED.
            flightStick, 
            new JoystickButton[]{
                AbstractButtonFactory.createButtonWithTrigger(
                    flightStick, 
                    port, 
                    _drive.buildCommand().stopWithX(),
                    AbstractButtonFactory.CONTROL.ON_TRUE
                    ),

                AbstractButtonFactory.createButtonWithTrigger(
                    flightStick, 
                    port, 
                    _drive.buildCommand().rotateAboutSpeaker(
                        () -> flightStick.getY(),
                        () -> flightStick.getX()
                    ),
                    AbstractButtonFactory.CONTROL.WHILE_TRUE
                ),

                AbstractButtonFactory.createButtonWithTrigger(
                    flightStick, 
                    port, 
                    _drive.buildCommand().resetRobotFront(),
                    AbstractButtonFactory.CONTROL.ON_TRUE
                ),

                AbstractButtonFactory.createButtonWithTrigger(
                    flightStick, 
                    port, 
                    _drive.buildCommand().resetRobotPoseSpeaker(),
                    AbstractButtonFactory.CONTROL.ON_TRUE
                )
            }
        );  

        _drive.setDefaultCommand(_drive.buildCommand().drive(
            () -> flightStick.getY(), 
            () -> flightStick.getX(), 
            () -> flightStick.getTwist())
        );
        
        return flightStick;
    }
}

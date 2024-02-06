package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.robot.hardware.HardwareConstants;

public class LEDs {
    CANdle _candle;
    static int _numLEDs = 8;
    LEDAnimation _currentAnimation;

    public enum LEDAnimation {
        ReadyToShoot(new LEDColor(0, 255, 0), null),
        PickedUpNote(null, new StrobeAnimation(255, 165, 0, 0, 0.5, _numLEDs));

        LEDColor _color;
        Animation _animation;

        LEDAnimation(LEDColor color, Animation animation) {
            _color = color;
            _animation = animation;
        }

        public LEDColor getColor() {
            return _color;
        }

        public Animation getAnimation() {
            return _animation;
        }
    }

    public LEDs() {
        _candle = new CANdle(HardwareConstants.CanIds.CANDLE_ID);
    }
}

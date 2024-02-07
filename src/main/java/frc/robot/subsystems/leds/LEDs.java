package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.robot.hardware.HardwareConstants;

public class LEDs {
    CANdle _candle;
    static int _numLEDs = 8;
    LEDAnimation _currentAnimation = LEDAnimation.None;
    boolean _prevHasNote;

    public enum LEDAnimation {
        None(null, null, 0),
        ReadyToShoot(new LEDColor(0, 255, 0), null, 0),
        PickedUpNote(null, new StrobeAnimation(255, 165, 0, 0, 0.5, _numLEDs), 3);

        LEDColor _color;
        Animation _animation;
        int _secondsToRun;

        LEDAnimation(LEDColor color, Animation animation, int secondsToRun) {
            _color = color;
            _animation = animation;
            _secondsToRun = secondsToRun;
        }

        public LEDColor getColor() {
            return _color;
        }

        public Animation getAnimation() {
            return _animation;
        }

        public int getSecondsToRun() {
            return _secondsToRun;
        }
    }

    public LEDs() {
        _candle = new CANdle(HardwareConstants.CanIds.CANDLE_ID);
    }

    public LEDAnimation getCurrentAnimation() {
        return _currentAnimation;
    }

    public void runAnimation(LEDAnimation animation) {
        if (animation != _currentAnimation) {
            _currentAnimation = animation;

            if (animation.getColor() == null) {
                _candle.clearAnimation(0);
                _candle.animate(animation.getAnimation());
            } else {
                LEDColor color = animation.getColor();
                _candle.setLEDs(color.getR(), color.getB(), color.getG());
            }
        }
    }

    public void stopAnimationIfTimedOut(int currentRunTimeIn20Ms) {
        // Stop the current animation if it has run for its time
        int animationTime = _currentAnimation.getSecondsToRun();
        if (animationTime > 0) {
            // We actually have a timeout
            if (currentRunTimeIn20Ms >= animationTime * 50) {
                // We have finished running, so stop the animation
                _candle.clearAnimation(0);
                _currentAnimation = LEDAnimation.None;
            }
        }
    }

    public boolean shouldRunHasNoteAnimation(boolean hasNote) {
        boolean ret = hasNote && !_prevHasNote;
        _prevHasNote = hasNote;
        return ret;
    }
}

package frc.robot.subsystems.leds;

import org.easymock.bytebuddy.dynamic.scaffold.TypeInitializer.None;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import frc.robot.HardwareConstants;

public class LEDs {
    CANdle _candle;
    static int _numLEDs = 8;
    LEDAnimation _currentAnimation = LEDAnimation.None;
    boolean _prevHasNote;
    boolean _ampReady = false;


    public enum LEDAnimation {
        None(null, null, 0),
        // RobotIdle(null, new FireAnimation(1.0, 0.02, _numLEDs, 0.01, 0.0), 0),
        RobotIdle(new LEDColor(255, 128, 0), null, 0),
        ReadyToShoot(new LEDColor(0, 255, 0), null, 3),
        PickedUpNote(null, new StrobeAnimation(255, 128, 0, 0, 0.5, _numLEDs), 3),
        ClimberAscending(new LEDColor(255, 215, 0), new LarsonAnimation(0, 234, 255, 0, 0.5, _numLEDs, BounceMode.Back, 5, 0), 3),
        AmpReady(null, new StrobeAnimation(204, 0, 255, 0, 0.5, _numLEDs), 3),
        //Change time on RobotDisabled
        RobotDisabled(null, new RainbowAnimation(100, 0.5, _numLEDs), 0);
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
        Logger.recordOutput("LEDS/currentAnimation", _currentAnimation);
        Logger.recordOutput("LEDS/requestedAnimation", animation);
        if (animation != _currentAnimation) {
            _currentAnimation = animation;

            if (animation.getColor() == null) {
                _candle.clearAnimation(0);
                _candle.setLEDs(0,0,0);
                _candle.animate(animation.getAnimation());
            } else if (animation.getAnimation() == null){
                LEDColor color = animation.getColor();
                _candle.clearAnimation(0);
                _candle.setLEDs(0,0,0);     
                _candle.setLEDs(color.getR(), color.getB(), color.getG());
            } else {
                _candle.clearAnimation(0);
                _candle.animate(animation.getAnimation());
                LEDColor color = animation.getColor();
                _candle.setLEDs(color.getR(), color.getB(), color.getG());
            }
            
        }
    }

    /**
     * returns true if the animation was stopped so we can reset the timer
     * @param currentRunTimeIn20Ms
     * @return
     */
    public boolean stopAnimationIfTimedOut(int currentRunTimeIn20Ms) {
        // Stop the current animation if it has run for its time
        int animationTime = _currentAnimation.getSecondsToRun();
        boolean wasAnimationStopped = false;
        if (animationTime > 0) {
            // We actually have a timeout
            if (currentRunTimeIn20Ms >= animationTime * 50) {
                Logger.recordOutput("LEDS/animationTimeout",true);
                wasAnimationStopped = true;
                if (_currentAnimation.getColor() == null) {
                _candle.clearAnimation(0);
                } else if (_currentAnimation.getAnimation() == null){
                    _candle.setLEDs(0, 0, 0);
                } else {
                    _candle.clearAnimation(0);
                    _candle.setLEDs(0, 0, 0);
                }
            _currentAnimation = LEDAnimation.None;
            }
            else{
                Logger.recordOutput("LEDS/animationTimeout",false);
            }
        }
        else{
            Logger.recordOutput("LEDS/animationTimeout",false);
        }
        return wasAnimationStopped;
    }

    public boolean shouldRunHasNoteAnimation(boolean hasNote) {
        boolean ret = hasNote && !_prevHasNote;
        _prevHasNote = hasNote;
        return ret;
    }
    
    public boolean getAmpReady() {
        return _ampReady;
    }

    public void setAmpReady(boolean isReady) {
        _ampReady = isReady;
    }
}

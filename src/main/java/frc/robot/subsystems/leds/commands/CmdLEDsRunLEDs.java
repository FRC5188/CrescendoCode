package frc.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.LEDAnimation;
import frc.robot.subsystems.shooter.Shooter;

public class CmdLEDsRunLEDs extends Command {
  private LEDs _leds;
  private Shooter _shooter;
  private Intake _intake;
  private int _timer;
  
  public CmdLEDsRunLEDs(LEDs leds, Shooter shooter, Intake intake) {
    _leds = leds;
    _shooter = shooter;
    _intake = intake;
  }

  @Override
  public void initialize() {
    _timer = 0;
  }

  @Override
  public void execute() {
    // Stop running the current animation if it's run for long enough
    boolean wasAnimationStopped = _leds.stopAnimationIfTimedOut(_timer);
    if(wasAnimationStopped){
      _timer = 0;
    }

    // // Start a new animation if we meet conditions
    if (DriverStation.isDisabled()) {
      // rainbow when DS is disabled
      _leds.runAnimation(LEDAnimation.RobotDisabled);
    } else if(_leds.shouldRunHasNoteAnimation(_intake.hasNote())){
            // if we do not have a note, then blink red
            _leds.runAnimation(LEDAnimation.PickedUpNote);
    } else if (_leds._isAutoAimEnabled()) {
          // the drive wants to use autoaim  
          if(_shooter.shooterInPosition() && _leds._isAutoAimReady()){
            // if the shooter is in position and the autoaim code says its ready
            // then signal to the drive we are ready
            _leds.runAnimation(LEDAnimation.ReadyToShoot);
          } else{
            // if we have a note but are not in position then we are not ready.
            _leds.runAnimation(LEDAnimation.SolidRed);
          }
     }
    // } else if (_leds.shouldRunHasNoteAnimation(_intake.hasNote())) {
    //   // Only run this animation one time, right when the intake first picks up a note
    //   // blinks orange when we pick up a note
    //   _leds.runAnimation(LEDAnimation.PickedUpNote);
    // }
    else{
      // solid orange when we are doing nothing
      _leds.runAnimation(LEDAnimation.RobotIdle);
    }

    // Increment the timer for another loop
    _timer++;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

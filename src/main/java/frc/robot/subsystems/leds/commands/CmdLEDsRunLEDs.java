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
    _leds.stopAnimationIfTimedOut(_timer);

    // Start a new animation if we meet conditions
    if (_shooter.isReady()) {
      // Run this animation when the shooter is ready
      _leds.runAnimation(LEDAnimation.ReadyToShoot);
    } else if (_leds.shouldRunHasNoteAnimation(_intake.hasNote())) {
      // Only run this animation one time, right when the intake first picks up a note
      _leds.runAnimation(LEDAnimation.PickedUpNote);
    // } else if (_leds.getAmpReady()) {
    //   // Runs when human player should activate Amp
    //   _leds.runAnimation(LEDAnimation.AmpReady);
    //   _leds.setAmpReady(false);
    } else if (DriverStation.isDisabled()) {
      // Rainbows while robot is disabled
      _leds.runAnimation(LEDAnimation.RobotDisabled);
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

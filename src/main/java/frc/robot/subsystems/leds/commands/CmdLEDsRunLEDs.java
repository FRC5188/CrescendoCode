package frc.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.LEDAnimation;
import frc.robot.subsystems.shooter.Shooter;

public class CmdLEDsRunLEDs extends Command {
  private LEDs _leds;
  private Shooter _shooter;
  private Intake _intake;
  private Climber _climber;
  private int _timer;
  
  /**
   * Runs LEDs for shooter, intake, and climber
   * @param leds
   * @param shooter
   * @param intake
   * @param climber
   */
  public CmdLEDsRunLEDs(LEDs leds, Shooter shooter, Intake intake, Climber climber) {
    _leds = leds;
    _shooter = shooter;
    _intake = intake;
    _climber = climber;
  }

  @Override
  /**
   * Initializes command and sets timer to 0
   */
  public void initialize() {
    _timer = 0;
  }

  @Override
  /**
   * Runs the corresponding LED sequence for each circumstance
   */
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
    } else if (_climber.isClimbing()) {
      // Runs when climber is active
      _leds.runAnimation(LEDAnimation.ClimberAscending);
    }

    // Increment the timer for another loop
    _timer++;
  }

  @Override
  /**
   * Ends the command if the sequence is interrupted or if commanded to do so
   */
  public void end(boolean interrupted) {}

  @Override
  /**
   * Returns false for isFinished, so loops back to the beginning and starts the sequences again
   */
  public boolean isFinished() {
    return false;
  }
}

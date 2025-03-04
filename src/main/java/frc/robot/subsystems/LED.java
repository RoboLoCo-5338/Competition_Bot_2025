package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {

  private final LEDIO io;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  public LED(LEDIO io) {
    this.io = io;
  }

  /** Updates the LED states and logs the current state. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LED", inputs);
  }

  /**
   * Creates an InstantCommand to set the given LEDPattern on the LEDs. This does not require any
   * periodic updates, so it can be used as a one-time command or as part of a larger command group.
   *
   * @param pattern The LEDPattern to set on the LEDs.
   * @return An InstantCommand that applies the given LEDPattern to the LED.
   */
  public Command setLEDPatternCommand(LEDPattern pattern) {
    return new InstantCommand(() -> io.setLEDPattern(pattern));
  }

  /**
   * Creates an InstantCommand to set a scrolling rainbow pattern on the LEDs.
   *
   * @return An InstantCommand that applies the scrolling rainbow pattern to the LED.
   */
  public Command setRainbowLEDCommand() {
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    LEDPattern scrollingRainbow =
        rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(2), LEDConstants.LED_SPACING);
    return new RunCommand(() -> io.setLEDPattern(scrollingRainbow));
  }

  /**
   * Creates an InstantCommand to set an LED pattern that represents progress.
   *
   * @param progress A double value representing the progress to be displayed, where 0.0 means no
   *     progress and 1.0 means full progress.
   * @return An InstantCommand that applies the progress mask pattern to the LED.
   */
  public Command setProgressMaskCommand(double progress) {
    LEDPattern pattern = LEDPattern.progressMaskLayer(() -> progress);
    return new InstantCommand(() -> io.setLEDPattern(pattern));
  }
}

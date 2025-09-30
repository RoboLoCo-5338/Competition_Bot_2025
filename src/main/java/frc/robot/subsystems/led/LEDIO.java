package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import org.littletonrobotics.junction.AutoLog;

public class LEDIO {

  @AutoLog
  public static class LedIOInputs {}

  /**
   * Updates inputs for the LED subsystem.
   *
   * @param inputs
   */
  public void updateInputs(LedIOInputs inputs) {}

  /**
   * Sets LED pattern
   *
   * @param pattern
   */
  public void setLEDPattern(LEDPattern pattern) {}

  /**
   * Changes the LED port
   *
   * @param port
   */
  public void changeLEDPort(int port) {}
}

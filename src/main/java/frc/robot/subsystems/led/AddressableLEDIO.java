package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.subsystems.led.LEDIO.LedIOInputs;

public class AddressableLEDIO implements LEDIO {
  //creates new LED connected to physical
  private AddressableLED m_led = new AddressableLED(LEDConstants.LED_PWM_PORT);
  //I think sets the number of leds in the strip
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);

  public AddressableLEDIO() {
    //sets number of leds in the strip
    m_led.setLength(m_ledBuffer.getLength());
    //empty buffer (no color yet)
    m_led.setData(m_ledBuffer);
    //starts the LED strip
    m_led.start();
  }

  /**
   * Updates the set of loggable inputs. There are no inputs for the LED subsystem, so this method
   * does nothing.
   */
  @Override
  public void updateInputs(LedIOInputs inputs) {
    // I don't think there are any inputs so this should just stay blank?
  }

  /**
   * Applies the given LED pattern to the LED strip.
   *
   * <p>This method applies the given pattern to the LED strip and then updates the LED strip's
   * data. This should be called every time the LEDs should be updated.
   *
   * @param pattern the pattern to apply to the LED strip
   */
  @Override
  public void setLEDPattern(LEDPattern pattern) {
    //changes the buffer
    pattern.applyTo(m_ledBuffer);
    //applies the buffer to the led
    m_led.setData(m_ledBuffer);
  }

  public void changeLEDPort(int port) {
    //changes the led port (I think this is wrong b/c we didn't setLength or setData or start this new LED)
    m_led.close();
    m_led = new AddressableLED(port);
  }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.Constants.LEDConstants;

public class AddressableLEDIO implements LEDIO {
  private final AddressableLED m_led = new AddressableLED(LEDConstants.LED_PWM_PORT);
  private final AddressableLEDBuffer m_ledBuffer =
      new AddressableLEDBuffer(LEDConstants.LED_LENGTH);

  public AddressableLEDIO() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    // I don't think there are any inputs so this should just stay blank?
  }

  @Override
  public void setLEDPattern(LEDPattern pattern) {
    pattern.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }
}

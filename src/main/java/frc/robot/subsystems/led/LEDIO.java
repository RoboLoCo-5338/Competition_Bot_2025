package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import org.littletonrobotics.junction.AutoLog;

public class LEDIO {

  public LEDIO() {
    final AddressableLEDBuffer mAddressableLEDBuffer = new AddressableLEDBuffer(0);
    final LEDIOAddressable mAddressable = new LEDIOAddressable();
  }

  @AutoLog
  public static class LedIOInputs {}

  public void updateInputs(LedIOInputs inputs) {}

  public void setLEDPattern(LEDPattern pattern) {}

  public void changeLEDPort(int port) {}

  public int setLength(int lengthLED){
    return lengthLED;
  }

  public AddressableLEDBuffer setData(AddressableLEDBuffer mAddressableLEDBuffer) {
    return mAddressableLEDBuffer; 
  }

  public void start(){}
}

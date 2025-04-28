package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import org.littletonrobotics.junction.AutoLog;

//just abstract interface
public interface LEDIO {
  //doesn't autolog anything
  @AutoLog
  public static class LedIOInputs {}

  public default void updateInputs(LedIOInputs inputs) {}

  public default void setLEDPattern(LEDPattern pattern) {}

  public default void changeLEDPort(int port) {}
}

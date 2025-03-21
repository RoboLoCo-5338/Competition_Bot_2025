package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {

  @AutoLog
  public static class LedIOInputs {}

  public default void updateInputs(LedIOInputs inputs) {}

  public default void setLEDPattern(LEDPattern pattern) {}

  public default void changeLEDPort(int port) {}
}

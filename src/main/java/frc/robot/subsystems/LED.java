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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LED", inputs);
  }

  public Command setLEDPattern(LEDPattern pattern) {
    return new InstantCommand(() -> io.setLEDPattern(pattern));
  }

  public Command setRainbowLED() {
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    LEDPattern scrollingRainbow =
        rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(2), LEDConstants.LED_SPACING);
    return new RunCommand(() -> io.setLEDPattern(scrollingRainbow));
  }

  public Command setProgressMask(double progress) {
    LEDPattern pattern = LEDPattern.progressMaskLayer(() -> progress);
    return new InstantCommand(() -> io.setLEDPattern(pattern));
  }
}

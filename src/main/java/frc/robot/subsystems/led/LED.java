package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class LED extends SubsystemBase {

  private final AddressableLED m_led;
  private final AddressableLEDBuffer buffer;

  public LED() {

    m_led = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(123);
    m_led.setLength(buffer.getLength());
    m_led.setData(buffer);
    m_led.start();
  }

  public InstantCommand turnGreen(BooleanSupplier inverted) {
  
    if (inverted.getAsBoolean()) {
      return new InstantCommand(
          () -> {
            LEDPattern red = LEDPattern.solid(Color.kRed);
            red.applyTo(buffer);
            m_led.setData(buffer);
          });
    }
    return new InstantCommand(
        () -> {
          LEDPattern green = LEDPattern.solid(Color.kGreen);
          green.applyTo(buffer);
          m_led.setData(buffer);
        });
  }

  public InstantCommand turnOff() {

    return new InstantCommand(
        () -> {
          LEDPattern off = LEDPattern.kOff;
          off.applyTo(buffer);
          m_led.setData(buffer);
        });
  }

  public InstantCommand goRainbow() {

    return new InstantCommand(
        () -> {
          LEDPattern rainbow = LEDPattern.rainbow(255, 128);
          LEDPattern scrollingRainbow =
              rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(2), LEDConstants.LED_SPACING);
          scrollingRainbow.applyTo(buffer);
          m_led.setData(buffer);
        });
  }

  public InstantCommand turnColor(Color color) {

    return new InstantCommand(
        () -> {
          LEDPattern colorPattern = LEDPattern.solid(color);
          colorPattern.applyTo(buffer);
          m_led.setData(buffer);
        });
  }

  /**
   * Creates an InstantCommand to set an LED pattern that represents progress.
   *
   * @param progress A double value representing the progress to be displayed, where 0.0 means no
   *     progress and 1.0 means full progress.
   * @return An InstantCommand that applies the progress mask pattern to the LED.
   */
  public static double getDistanceFromBarge(Drive drive) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
      return (-drive.getPose().getX() + 8.272272) / LEDConstants.BARGE_RANGE;
    } else {
      return (drive.getPose().getX() - 9.27) / LEDConstants.BARGE_RANGE;
    }
  }

  public Trigger isCloseToBarge(Drive drive) {
    return new Trigger(() -> getDistanceFromBarge(drive) < -0.5);
  }

  public Command setBargeIndicator(Drive drive, Elevator elevator) {
    return new RunCommand(
        () -> {
          var progress = LEDPattern.progressMaskLayer(() -> getDistanceFromBarge(drive));

          LEDPattern.gradient(GradientType.kContinuous, Color.kCyan, Color.kYellow)
              .mask(progress)
              .applyTo(buffer);

          m_led.setData(buffer);
        });
  }
}

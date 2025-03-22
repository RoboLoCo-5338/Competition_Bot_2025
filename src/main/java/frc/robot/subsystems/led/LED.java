package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.MetersPerSecond;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.Logger;

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

  


  /**
   * Creates an InstantCommand to set an LED pattern that represents progress.
   *
   * @param progress A double value representing the progress to be displayed, where 0.0 means no
   *     progress and 1.0 means full progress.
   * @return An InstantCommand that applies the progress mask pattern to the LED.
   */

  public SequentialCommandGroup flashGreen() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              LEDPattern green = LEDPattern.solid(Color.kGreen);
              green.applyTo(buffer);
              m_led.setData(buffer);
            }),
        new WaitCommand(0.5),
        new InstantCommand(
            () -> {
              LEDPattern off = LEDPattern.kOff;
              off.applyTo(buffer);
              m_led.setData(buffer);
            }),
        new WaitCommand(5),
        new InstantCommand(
            () -> {
              LEDPattern rainbow = LEDPattern.rainbow(255, 128);
              LEDPattern scrollingRainbow =
                  rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(2), LEDConstants.LED_SPACING);
              scrollingRainbow.applyTo(buffer);
              m_led.setData(buffer);
            })
      );
            


  }
  public static double getDistanceFromBarge(Drive drive) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
      return (-drive.getPose().getX() + .272272) / LEDConstants.BARGE_RANGE;
    } else {
      return (drive.getPose().getX() - 10.27) / LEDConstants.BARGE_RANGE;
    }
  }

  public Command setBargeIndicator(Drive drive, Elevator elevator) {
    return new InstantCommand(
        () -> {
          if (getDistanceFromBarge(drive) < 1.0) {
            var progress = LEDPattern.progressMaskLayer(() -> getDistanceFromBarge(drive));
          
                LEDPattern.gradient(GradientType.kContinuous, Color.kCyan, Color.kYellow)
                    .mask(progress).applyTo(buffer);
                  
                m_led.setData(buffer);
          } else {
            LEDPattern rainbow = LEDPattern.rainbow(255, 128);
            LEDPattern scrollingRainbow =
                rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(2), LEDConstants.LED_SPACING);
            scrollingRainbow.applyTo(buffer);
            m_led.setData(buffer);
          }
        });
  }
}

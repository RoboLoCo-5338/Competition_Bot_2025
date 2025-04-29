package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;

public class LED extends SubsystemBase {
  //Wow, we don't even use AddressableLEDIO, surprising
  private final AddressableLED m_led;
  private final AddressableLEDBuffer buffer;

  public LED() {
    //creates new LED connected to physical with buffer 300 (number of LEDS in the strip)
    m_led = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(300);
    m_led.setLength(buffer.getLength());
    //no led color
    m_led.setData(buffer);
    //starts the LED strip
    m_led.start();
  }

  public Command alignEndFlash(boolean canceled) {
    return new ScheduleCommand(
        new SequentialCommandGroup(
                turnColor(canceled ? Color.kRed : Color.kGreen).withTimeout(0.3),
                turnOff(),
                new WaitCommand(0.3),
                turnColor(canceled ? Color.kRed : Color.kGreen).withTimeout(0.3),
                turnOff(),
                new WaitCommand(0.3),
                turnColor(canceled ? Color.kRed : Color.kGreen))
            .withTimeout(0.3));
  }

  public Command pulseBlue() {
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    LEDPattern pulsingBlue = blue.breathe(Seconds.of(5));
    return new RunCommand(
      //continuously applies the pattern to the buffer
        () -> {
          pulsingBlue.applyTo(buffer);
          m_led.setData(buffer);
        },
        this);
  }

  public Command turnOff() {
    return new InstantCommand(
      //makes the LED no color
        () -> {
          LEDPattern off = LEDPattern.kOff;
          off.applyTo(buffer);
          m_led.setData(buffer);
        },
        this);
  }

  public Command goRainbow() {
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    LEDPattern scrollingRainbow =
    //makes the rainbow move along the strip 0.3 m/s
        rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.3), LEDConstants.LED_SPACING);
    return new InstantCommand(
        () -> {
          scrollingRainbow.applyTo(buffer);
          m_led.setData(buffer);
        },
        this);
  }

  public Command turnColor(Color color) {
    return new RunCommand(
        () -> {
          LEDPattern colorPattern = LEDPattern.solid(color);
          colorPattern.applyTo(buffer);
          m_led.setData(buffer);
        },
        this);
  }

  /**
   * Creates an InstantCommand to set an LED pattern that represents progress.
   *
   * @param progress A double value representing the progress to be displayed, where 0.0 means no
   *     progress and 1.0 means full progress.
   * @return An InstantCommand that applies the progress mask pattern to the LED.
   */
  public static double getDistanceFromBarge(Drive drive) {
    //gets distance from barge
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
      return (-drive.getPose().getX() + 8.272272);
    } else {
      return (drive.getPose().getX() - 9.27);
    }
  }
  //Trigger b/c it makes it so that we don't have to check it in periodic(), it is similar to button triggers
  public Trigger isCloseToBarge(Drive drive) {
    //checks if the robot distance is between 1.45 and 0.6 meters from barge
    return new Trigger(
        () -> getDistanceFromBarge(drive) < 1.45 && getDistanceFromBarge(drive) > 0.60);
  }

  public Trigger isCriticalToBarge(Drive drive) {
    //checks if the robot distance is less than 0.6 meters from barge
    return new Trigger(() -> getDistanceFromBarge(drive) < 0.60);
  }

  public SequentialCommandGroup sendBargeIndicator(CommandXboxController controller) {
    //rumbles controllers
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              controller.setRumble(RumbleType.kBothRumble, 1.0);
            }),
        new WaitCommand(0.5),
        new InstantCommand(
            () -> {
              controller.setRumble(RumbleType.kBothRumble, 0.0);
            }));
  }
}

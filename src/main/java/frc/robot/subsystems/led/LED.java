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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class LED extends SubsystemBase {

  private final AddressableLED m_led;
  private final AddressableLEDBuffer buffer;

  public LED() {

    m_led = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(300);
    m_led.setLength(buffer.getLength());
    m_led.setData(buffer);
    m_led.start();
  }

  public InstantCommand flashGreen() {
    return new InstantCommand(
        () -> {
          new SequentialCommandGroup(
                  turnGreen(),
                  new WaitCommand(0.3),
                  turnOff(),
                  new WaitCommand(0.3),
                  turnGreen(),
                  new WaitCommand(0.3),
                  turnOff(),
                  new WaitCommand(0.3),
                  turnGreen())
              .schedule();
        });
  }

  public InstantCommand rainbowSet(boolean enabled) {
    return new InstantCommand(
        () -> {
          RobotContainer.doRainbow = enabled;
        });
  }

  public InstantCommand flashBlue() {
    return new InstantCommand(
        () -> {
          new SequentialCommandGroup(
                  rainbowSet(false),
                  turnColor(Color.kBlue),
                  new WaitCommand(0.15),
                  turnOff(),
                  new WaitCommand(0.15),
                  turnColor(Color.kBlue),
                  new WaitCommand(0.15),
                  turnOff(),
                  new WaitCommand(0.15),
                  turnColor(Color.kBlue),
                  new WaitCommand(0.15),
                  turnOff(),
                  new WaitCommand(0.15),
                  turnColor(Color.kBlue),
                  new WaitCommand(1.5),
                  rainbowSet(true))
              .schedule();
        });
  }

  public InstantCommand turnGreen() {

    return new InstantCommand(
        () -> {
          if (DriveConstants.canceled) {

            LEDPattern red = LEDPattern.solid(Color.kRed);
            red.applyTo(buffer);
            m_led.setData(buffer);
          } else {
            LEDPattern green = LEDPattern.solid(Color.kGreen);
            green.applyTo(buffer);
            m_led.setData(buffer);
          }
        });
  }

  public RunCommand pulseBlue() {
    LEDPattern blue = LEDPattern.solid(Color.kBlue);

    LEDPattern pulsingBlue = blue.breathe(Seconds.of(5));
    return new RunCommand(
        () -> {
          pulsingBlue.applyTo(buffer);
          m_led.setData(buffer);
        },
        this);
  }

  public InstantCommand turnOff() {

    return new InstantCommand(
        () -> {
          LEDPattern off = LEDPattern.kOff;
          off.applyTo(buffer);
          m_led.setData(buffer);
        });
  }

  public RunCommand goRainbow() {
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    LEDPattern scrollingRainbow =
        rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.3), LEDConstants.LED_SPACING);
    return new RunCommand(
        () -> {
          scrollingRainbow.applyTo(buffer);
          m_led.setData(buffer);
        },
        this);
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
      return (-drive.getPose().getX() + 8.272272);
    } else {
      return (drive.getPose().getX() - 9.27);
    }
  }

  public Trigger isCloseToBarge(Drive drive) {
    return new Trigger(
        () -> {
          return Math.abs(getDistanceFromBarge(drive)) < 3.00
              && Math.abs(getDistanceFromBarge(drive)) > 0.60;
        });
  }

  public Trigger isCriticalToBarge(Drive drive) {
    return new Trigger(() -> getDistanceFromBarge(drive) < 0.60);
  }

  public SequentialCommandGroup sendBargeIndicator(CommandXboxController controller) {
    return new SequentialCommandGroup(
        flashBlue(),
        new InstantCommand(
            () -> {
              System.out.println("bargei indicator");
              controller.setRumble(RumbleType.kRightRumble, 1.0);
            }),
        new WaitCommand(1.0),
        new InstantCommand(
            () -> {
              // controller.setRumble(RumbleType.kRightRumble, 0.0);
            }));
  }
  // public Command setBargeIndicator(Drive drive, Elevator elevator) {
  //   return new RunCommand(
  //       () -> {

  //         LEDPattern.solid(Color.kWhite)
  //             .applyTo(buffer);

  //         m_led.setData(buffer);
  //       });
  // }
}

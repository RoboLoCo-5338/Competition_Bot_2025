package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.IO; // Ensure this matches the actual package of IO

public class ArmIO extends IO{

  @AutoLog
  public static class ArmIOInputs extends IO.IOInputs {
    public double armPosition = 0.0;
    public double armVelocity = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrent = 0.0;
    public boolean armConnected = false;
    public double armTemperature = 0.0;
  }

  /**
   * Updates the set of loggable inputs for the arm subsystem. This function updates the following
   * inputs:
   *
   * <ul>
   *   <li>{@code armConnected}: Whether the arm motor is connected
   *   <li>{@code armPosition}: The position of the arm motor in radians
   *   <li>{@code armVelocity}: The velocity of the arm motor in radians per second
   *   <li>{@code armAppliedVolts}: The voltage applied to the arm motor in volts
   *   <li>{@code armCurrent}: The current drawn by the arm motor in amps
   * </ul>
   */
  public void updateInputs(ArmIOInputs inputs) {}

  /**
   * Sets the position of the arm in radians. This method is "fire-and-forget" in the sense that it
   * will not block or wait for the arm to reach the desired position. If you want to verify that
   * the arm has reached the desired position, you must poll the {@link ArmIOInputs#armPosition}
   * loggable input. This method is intended to be used with position control, not velocity control.
   * If you want to set the velocity of the arm, use {@link #setArmVelocity(double)} instead.
   *
   * @param position the desired position in radians
   */
  public void setArmPosition(double position) {}

  /**
   * Sets the velocity of the arm in radians per second. This method is "fire-and-forget" in the
   * sense that it will not block or wait for the arm to reach the desired velocity. If you want to
   * verify that the arm has reached the desired velocity, you must poll the velocity using {@link
   * #updateInputs(ArmIOInputs)}.
   *
   * @param velocityRadPerSec The velocity of the arm in radians per second.
   */
  public void setArmVelocity(double velocity) {}

  public double getArmPosition(ArmIOInputs inputs) {
    return 0.0;
  }

  public void armOpenLoop(Voltage voltage) {}
}

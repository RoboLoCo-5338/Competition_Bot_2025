package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.SysIDIO;
import frc.robot.subsystems.SysIDIO.SysIDIOInputs;
import org.littletonrobotics.junction.AutoLog;

public class ArmIO extends SysIDIO<ArmIOInputsAutoLogged> {

  @AutoLog
  public static class ArmIOInputs extends SysIDIOInputs {
    public double armAppliedVolts = 0.0;
    public double armCurrent = 0.0;
    public boolean armConnected = false;
    public double armTemperature = 0.0;
  }
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

  public void openLoop(Voltage voltage) {}
}

package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.SysIDIO;
import frc.robot.subsystems.SysIDIO.SysIDIOInputs;
import org.littletonrobotics.junction.AutoLog;

public class ElevatorIO extends SysIDIO<ElevatorIOInputsAutoLogged> {
  @AutoLog
  public static class ElevatorIOInputs extends SysIDIOInputs {
    public double elevator1Position = 0.0;
    public double elevator1Velocity = 0.0;
    public double elevator1AppliedVolts = 0.0;
    public double elevator1CurrentAmps = 0.0;
    public boolean elevator1Connected = false;
    public double elevator1Temperature = 0.0;

    public double elevator2Position = 0.0;
    public double elevator2Velocity = 0.0;
    public double elevator2AppliedVolts = 0.0;
    public double elevator2CurrentAmps = 0.0;
    public boolean elevator2Connected = false;
    public double elevator2Temperature = 0.0;
  }

  /**
   * Sets the target velocity for both elevator motors.
   *
   * <p>This method sends a velocity control request to both elevator motors to move them at the
   * specified velocity.
   *
   * @param velocity The target velocity in meters per second for the elevator motors.
   */
  public void setElevatorVelocity(double velocity) {}

  /**
   * Sets the target position for both elevator motors.
   *
   * <p>This method sends a position control request to both elevator motors to move them to the
   * specified position.
   *
   * @param position The target position in meters for the elevator motors.
   */
  public void setElevatorPosition(double position, int slot) {}

  /**
   * Gets the current measurement from the laser can sensor.
   *
   * <p>This method returns the current measurement in millimeters from the laser can sensor, or -1
   * if the measurement is invalid or not available.
   *
   * @return The current measurement in millimeters from the laser can sensor, or -1 if the
   *     measurement is invalid or not available.
   */
  public int getLaserCanMeasurement() {
    return -1;
  }

  public void elevatorOpenLoop(Voltage voltage) {}

  @Override
  public void openLoop(Voltage voltage) {}
}

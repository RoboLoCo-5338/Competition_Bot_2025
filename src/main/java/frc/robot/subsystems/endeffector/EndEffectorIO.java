package frc.robot.subsystems.endeffector;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    //auto logged stuff
    public double endEffectorVelocity = 0.0;
    public double endEffectorAppliedVolts = 0.0;
    public double endEffectorCurrentAmps = 0.0;
    public boolean endEffectorConnected = false;
    public double endEffectorDistance1 = -1;
    public double endEffectorDistance2 = -1;
    public double endEffectorTemperature = 0.0;
    public double endEffectorPosition = 0.0;
  }

  /**
   * Updates inputs for the end effector subsystem.
   * @param inputs
   */
  public void updateInputs(EndEffectorIOInputs inputs) {}

  /**
   * Sets end effector velocity
   * @param velocity m/s
   */
  public void setEndEffectorVelocity(double velocity) {}

  /**
   * Sets end effector speed
   * @param speed (-1 to 1)
   */
  public void setEndEffectorSpeed(double speed) {}

  /**
   * Gets lasercan1 measurement
   * @return
   */
  public int getLaserCanMeasurement1() {
    return -1;
  }

  /**
   * Gets lasercan2 measurement
   * @return
   */
  public int getLaserCanMeasurement2() {
    return -1;
  }

  /**
   * Open loop control for the end effector motor using voltage
   * @param voltage
   */
  public void endEffectorOpenLoop(Voltage voltage) {}
}

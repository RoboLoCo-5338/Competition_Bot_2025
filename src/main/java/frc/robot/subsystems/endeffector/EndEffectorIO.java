package frc.robot.subsystems.endeffector;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public double endEffectorVelocity = 0.0;
    public double endEffectorAppliedVolts = 0.0;
    public double endEffectorCurrentAmps = 0.0;
    public boolean endEffectorConnected = false;
    public double endEffectorDistance1 = -1;
    public double endEffectorDistance2 = -1;
    public double endEffectorTemperature = 0.0;
    public double endEffectorPosition = 0.0;
  }

  public void updateInputs(EndEffectorIOInputs inputs) {}

  public void setEndEffectorVelocity(double velocity) {}

  public void setEndEffectorSpeed(double speed) {}

  public int getLaserCanMeasurement1() {
    return -1;
  }

  public int getLaserCanMeasurement2() {
    return -1;
  }

  public void endEffectorOpenLoop(Voltage voltage) {}
}

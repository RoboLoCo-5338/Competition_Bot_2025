package frc.robot.subsystems.endeffector;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.SysIDIO;
import frc.robot.subsystems.SysIDIO.SysIDIOInputs;
import org.littletonrobotics.junction.AutoLog;

public class EndEffectorIO extends SysIDIO<EndEffectorIOInputsAutoLogged> {
  @AutoLog
  public static class EndEffectorIOInputs extends SysIDIOInputs {
    public double endEffectorAppliedVolts = 0.0;
    public double endEffectorCurrentAmps = 0.0;
    public double endEffectorDistance1 = -1;
    public double endEffectorDistance2 = -1;
    public double endEffectorTemperature = 0.0;
  }

  public void setEndEffectorVelocity(double velocity) {}

  public void setEndEffectorSpeed(double speed) {}

  public int getLaserCanMeasurement1() {
    return -1;
  }

  public int getLaserCanMeasurement2() {
    return -1;
  }

  public double getEndEffectorVelocity() {
    return 0.0;
  }

  public void openLoop(Voltage voltage) {}
}

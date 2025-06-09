package frc.robot.subsystems;

import edu.wpi.first.units.measure.Voltage;

public abstract class SysIDIO<S extends SysIDIO.SysIDIOInputs> extends IO<S> {
  public abstract void openLoop(Voltage voltage);

  public static class SysIDIOInputs extends IOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
  }
}

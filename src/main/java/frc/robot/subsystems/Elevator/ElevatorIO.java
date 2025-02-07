package frc.robot.subsystems.Elevator;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double elevator1Position = 0.0;
    public double elevator1Velocity = 0.0;
    public double elevator1AppliedVolts = 0.0;
    public double elevator1CurrentAmps = 0.0;
    public boolean elevator1Connected = false;

    public double elevator2Position = 0.0;
    public double elevator2Velocity = 0.0;
    public double elevator2AppliedVolts = 0.0;
    public double elevator2CurrentAmps = 0.0;
    public boolean elevator2Connected = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setElevatorVelocity(double velocity) {}

  public default void setElevatorPosition(double position) {}

  public default int getLaserCanMeasurement() {
    return -1;
  }
}

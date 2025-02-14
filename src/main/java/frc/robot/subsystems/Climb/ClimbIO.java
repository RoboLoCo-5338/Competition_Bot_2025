package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  @AutoLog
  public static class ClimbIOInputs {
    public double climbPosition = 0.0;
    public double climbVelocityRadPerSec = 0.0;
    public double climbAppliedVolts = 0.0;
    public double climbCurrentAmps = 0.0;
    public boolean climbConnected = false;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setClimbVelocity(double velocity) {}

  public default void setClimbPosition(double position) {}
}

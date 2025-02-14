package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

  @AutoLog
  public static class EndEffectorIOInputs {
    public double endEffectorVelocity = 0.0;
    public double endEffectorAppliedVolts = 0.0;
    public double endEffectorCurrentAmps = 0.0;
    public boolean endEffectorConnected = false;
    public double endEffectorDistance1 = -1;
    public double endEffectorDistance2 = -1;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setEndEffectorVelocity(double velocity) {}
}

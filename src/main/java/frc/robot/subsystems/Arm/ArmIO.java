package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public double armPosition = 0.0;
    public double armVelocity = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrent = 0.0;
    public boolean armConnected = false;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmPosition(double position) {}

  public default void setArmVelocity(double velocity) {}
}

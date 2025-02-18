package frc.robot.subsystems.groundintake;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
  @AutoLog
  public static class GroundIntakeIOInputs {
    public boolean armMotorConnected = false;
    public double armPositionRad = 0.0;
    public double armVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrentAmps = 0.0;

    public boolean intakeMotorConnected = false;
    public double intakeVelocityRadPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
  }

  public default void updateInputs(GroundIntakeIOInputs inputs) {}

  public default void setArmVelocity(double velocityRadPerSec) {}

  public default void setArmPosition(double position) {}

  public default void setIntakeVelocity(double velocityRadPerSec) {}
}

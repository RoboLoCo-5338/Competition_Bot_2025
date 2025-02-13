package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ClimbConstants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  final TalonFX climbMotor =
      new TalonFX(ClimbConstants.CLIMB_MOTOR_ID, TunerConstants.kCANBus.getName());

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

  /**
   * Returns a TalonFXConfiguration that is used to configure the climb motor's control loop and
   * current limit. The control loop is configured with the P, I, D, and S gains from
   * ClimbConstants. The current limit is set to 40 amps, but this should be changed later.
   *
   * @return a TalonFXConfiguration for the climb motor
   */
  default TalonFXConfiguration getConfiguration() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = ClimbConstants.CLIMB_kP;
    config.Slot0.kI = ClimbConstants.CLIMB_kI;
    config.Slot0.kD = ClimbConstants.CLIMB_kD;
    config.Slot0.kS = ClimbConstants.CLIMB_kS;

    var currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimitEnable = true;
    // TODO change this later
    currentConfig.StatorCurrentLimit = 40;
    config.CurrentLimits = currentConfig;
    return config;
  }
}

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ClimbConstants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  final TalonFX climbMotor =
      new TalonFX(ClimbConstants.CLIMB_MOTOR_ID, TunerConstants.kCANBus.getName());

  final PositionVoltage climbPositionRequest = new PositionVoltage(0.0);
  final VelocityVoltage climbVelocityRequest = new VelocityVoltage(0.0);

  @AutoLog
  public static class ClimbIOInputs {
    public double climbPosition = 0.0;
    public double climbVelocityRadPerSec = 0.0;
    public double climbAppliedVolts = 0.0;
    public double climbCurrentAmps = 0.0;
    public boolean climbConnected = false;
  }

  /**
   * Updates the set of loggable inputs from the TalonFX. This function is called by the periodic
   * method of the Climb subsystem. The inputs are updated as follows:
   *
   * <ul>
   *   <li>climbConnected is set to true if the TalonFX is OK and false otherwise.
   *   <li>climbAppliedVolts is set to the current voltage of the motor.
   *   <li>climbCurrentAmps is set to the current current draw of the motor.
   *   <li>climbPosition is set to the current position of the climb in radians.
   *   <li>climbVelocityRadPerSec is set to the current velocity of the climb in radians per second.
   * </ul>
   *
   * @param inputs the inputs to update
   */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /**
   * Sets the velocity of the climb in radians per second. This function runs the motor in voltage
   * control mode and sets the voltage to the value required to achieve the desired velocity.
   *
   * @param velocity the desired velocity in radians per second
   */
  public default void setClimbVelocity(double velocity) {}

  /**
   * Sets the position of the climb in radians. This function runs the motor in position control
   * mode and sets the target position to the value specified by the input parameter.
   *
   * @param position the desired position in radians
   */
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
    config.Slot0.kV = 0.0;

    var currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimitEnable = true;
    // TODO change this later
    currentConfig.StatorCurrentLimit = ClimbConstants.CLIMB_MOTOR_CURRENT_LIMIT;
    config.CurrentLimits = currentConfig;
    return config;
  }

  public default void disableMotor() {
    climbMotor.stopMotor();
  }
}

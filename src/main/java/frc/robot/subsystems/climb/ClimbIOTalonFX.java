package frc.robot.subsystems.Climb;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ClimbConstants;
import frc.robot.generated.TunerConstants;

public class ClimbIOTalonFX implements ClimbIO {

  private final TalonFX climbMotor;

  private final PositionVoltage climbPositionRequest = new PositionVoltage(0.0);
  private final VelocityVoltage climbVelocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<Angle> climbPosition;
  private final StatusSignal<AngularVelocity> climbVelocity;
  private final StatusSignal<Voltage> climbAppliedVolts;
  private final StatusSignal<Current> climbCurrent;

  private final Debouncer climbConnected = new Debouncer(0.5);

  public ClimbIOTalonFX() {
    climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID, TunerConstants.kCANBus.getName());

    climbMotor.getConfigurator().apply(getConfiguration());

    climbPosition = climbMotor.getPosition();
    climbVelocity = climbMotor.getVelocity();
    climbAppliedVolts = climbMotor.getMotorVoltage();
    climbCurrent = climbMotor.getStatorCurrent();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, climbPosition, climbVelocity, climbAppliedVolts, climbCurrent));

    ParentDevice.optimizeBusUtilizationForAll(climbMotor);
  }

  /**
   * Updates the set of loggable inputs from the TalonFX. This function is called by the periodic
   * method of the Climb subsystem. The inputs are updated as follows:
   *
   * <ul>
   *   <li>climbConnected is set to true if the TalonFX is OK and false otherwise.
   *   <li>climbAppliedVolts is set to the current voltage of the motor.
   *   <li>climbCurrentAmps is set to the current current draw of the motor.
   *   <li>climbPosition is set to the current position of the motor in radians.
   *   <li>climbVelocityRadPerSec is set to the current velocity of the motor in radians per second.
   * </ul>
   *
   * @param inputs the inputs to update
   */
  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    var climbStatus =
        BaseStatusSignal.refreshAll(
            climbPosition, climbAppliedVolts, climbAppliedVolts, climbCurrent);

    inputs.climbConnected = climbConnected.calculate(climbStatus.isOK());
    inputs.climbAppliedVolts = climbAppliedVolts.getValueAsDouble();
    inputs.climbCurrentAmps = climbCurrent.getValueAsDouble();
    inputs.climbPosition = Units.rotationsToRadians(climbPosition.getValueAsDouble());
    inputs.climbVelocityRadPerSec = Units.rotationsToRadians(climbPosition.getValueAsDouble());
  }

  /**
   * Sets the velocity of the climb motor in radians per second. This function runs the motor in
   * voltage control mode and sets the voltage to the value required to achieve the desired
   * velocity.
   *
   * @param velocity the desired velocity in radians per second
   */
  @Override
  public void setClimbVelocity(double velocity) {
    climbMotor.setControl(climbVelocityRequest.withVelocity(velocity));
  }

  /**
   * Sets the position of the climb motor in radians. This function runs the motor in position
   * control mode and sets the target position to the value specified by the input parameter.
   *
   * @param position the desired position in radians
   */
  @Override
  public void setClimbPosition(double position) {
    climbMotor.setControl(climbPositionRequest.withPosition(position));
  }
}
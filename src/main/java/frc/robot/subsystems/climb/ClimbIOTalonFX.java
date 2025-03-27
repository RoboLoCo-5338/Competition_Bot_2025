package frc.robot.subsystems.climb;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

public class ClimbIOTalonFX implements ClimbIO {

  private final TalonFX climbMotor;

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

  @Override
  public void setClimbVelocity(double velocity) {
    climbMotor.setControl(
        climbVelocityRequest.withVelocity(
            Units.radiansToRotations(velocity * ClimbConstants.GEARING)));
  }

  @Override
  public void setClimbPosition(double position) {
    climbMotor.setControl(
        climbPositionRequest.withPosition(
            Units.radiansToRotations(position * ClimbConstants.GEARING)));
  }
}

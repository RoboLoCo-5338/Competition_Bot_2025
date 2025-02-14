package frc.robot.subsystems.Arm;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ArmConstants;

public class ArmIOTalonFX implements ArmIO {

  private final PositionVoltage armPositionRequest = new PositionVoltage(0.0);
  private final VelocityVoltage armVelocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<Angle> armPosition;
  private final StatusSignal<AngularVelocity> armVelocity;
  private final StatusSignal<Voltage> armAppliedVolts;
  private final StatusSignal<Current> armCurrent;

  private final Debouncer armConnectedDebouncer = new Debouncer(0.5);

  private final TalonFX armMotor;

  public ArmIOTalonFX() {
    armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);
    armMotor.getConfigurator().apply(getArmConfig());

    armPosition = armMotor.getPosition();
    armVelocity = armMotor.getVelocity();
    armAppliedVolts = armMotor.getMotorVoltage();
    armCurrent = armMotor.getStatorCurrent();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, armPosition, armVelocity, armAppliedVolts, armCurrent));
    ParentDevice.optimizeBusUtilizationForAll(armMotor);
  }

  private TalonFXConfiguration getArmConfig() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // TODO look at this - it requires us to do something special
    // DO NOT FORGET, ARM WILL NOT WORK IF WE IGNORE THIS
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kP = ArmConstants.ARM_MOTOR_KP;
    config.Slot0.kI = ArmConstants.ARM_MOTOR_KI;
    config.Slot0.kD = ArmConstants.ARM_MOTOR_KD;
    config.Slot0.kG = ArmConstants.ARM_MOTOR_KG;
    config.Slot0.kV = ArmConstants.ARM_MOTOR_KV;

    var currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimitEnable = true;
    // TODO CHANGE THIS OR TORQUE MAY BE LIMITED/TOO HIGH
    currentConfig.StatorCurrentLimit = 40;
    config.CurrentLimits = currentConfig;
    return config;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    var motorstatus =
        BaseStatusSignal.refreshAll(armPosition, armVelocity, armAppliedVolts, armCurrent);

    inputs.armConnected = armConnectedDebouncer.calculate(motorstatus.isOK());
    inputs.armPosition = Units.rotationsToRadians(armPosition.getValueAsDouble());
    inputs.armVelocity = Units.rotationsToRadians(armVelocity.getValueAsDouble());
    inputs.armAppliedVolts = armAppliedVolts.getValueAsDouble();
    inputs.armCurrent = armCurrent.getValueAsDouble();
  }

  @Override
  public void setArmPosition(double position) {
    armMotor.setControl(armPositionRequest.withPosition(position));
  }

  @Override
  public void setArmVelocity(double velocity) {
    armMotor.setControl(armVelocityRequest.withVelocity(velocity));
  }
}

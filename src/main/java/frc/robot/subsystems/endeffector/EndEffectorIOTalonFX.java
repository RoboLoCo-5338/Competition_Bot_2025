package frc.robot.subsystems.endeffector;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EndEffectorSimConstants;

public class EndEffectorIOTalonFX extends EndEffectorIO {

  private final StatusSignal<AngularVelocity> endEffectorVelocity;
  private final StatusSignal<Voltage> endEffectorAppliedVolts;
  private final StatusSignal<Current> endEffectorCurrent;
  private final StatusSignal<Temperature> endEffectorTemperature;
  private final StatusSignal<Integer> endEffectorVersion;
  private final StatusSignal<Angle> endEffectorPosition;

  private final Debouncer effectorDebouncer = new Debouncer(0.5);

  private final LaserCan LcEffector1;
  private final LaserCan LcEffector2;

  public final TalonFX endEffectorMotor =
      new TalonFX(EndEffectorConstants.EFFECTORID, TunerConstants.DrivetrainConstants.CANBusName);
  final VelocityVoltage endEffectorVelocityRequest = new VelocityVoltage(0.0);
  final VoltageOut endEffectorOpenLoop = new VoltageOut(0.0);

  public EndEffectorIOTalonFX() {

    endEffectorVelocity = endEffectorMotor.getVelocity();
    endEffectorAppliedVolts = endEffectorMotor.getMotorVoltage();
    endEffectorCurrent = endEffectorMotor.getStatorCurrent();
    endEffectorTemperature = endEffectorMotor.getDeviceTemp();
    endEffectorVersion = endEffectorMotor.getVersion();
    endEffectorPosition = endEffectorMotor.getPosition();

    endEffectorMotor.getConfigurator().apply(getEndEffectorConfiguration());

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                endEffectorVelocity,
                endEffectorAppliedVolts,
                endEffectorCurrent,
                endEffectorTemperature));

    ParentDevice.optimizeBusUtilizationForAll(endEffectorMotor);

    LcEffector1 = new LaserCan(EndEffectorConstants.LASERCAN_1ID);
    LcEffector2 = new LaserCan(EndEffectorConstants.LASERCAN_2ID);
    try {
      LcEffector1.setRangingMode(LaserCan.RangingMode.SHORT);
      LcEffector2.setRangingMode(LaserCan.RangingMode.SHORT);
      LcEffector1.setRegionOfInterest(
          new LaserCan.RegionOfInterest(2, 2, 2, 2)); // needs to be changed
      LcEffector2.setRegionOfInterest(
          new LaserCan.RegionOfInterest(2, 2, 2, 2)); // also needs to be changed
      LcEffector1.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
      LcEffector2.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);

    } catch (Exception e) {
      System.out.println("Error: " + e);
    }
  }

  public TalonFXConfiguration getEndEffectorConfiguration() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = EndEffectorConstants.EFFECTOR_KP;
    config.Slot0.kI = EndEffectorConstants.EFFECTOR_KI;
    config.Slot0.kD = EndEffectorConstants.EFFECTOR_KD;
    config.Slot0.kG = EndEffectorConstants.EFFECTOR_KG;
    config.Slot0.kV = EndEffectorConstants.EFFECTOR_KV;

    var currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimit = EndEffectorConstants.EFFECTOR_CURRENT_LIMIT;
    config.CurrentLimits = currentConfig;
    return config;
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    var motor1Status =
        BaseStatusSignal.refreshAll(
            endEffectorVelocity,
            endEffectorCurrent,
            endEffectorAppliedVolts,
            endEffectorPosition,
            endEffectorTemperature);

    inputs.endEffectorConnected = effectorDebouncer.calculate(motor1Status.isOK());
    inputs.endEffectorDistance1 = getLaserCanMeasurement1();
    inputs.endEffectorDistance2 = getLaserCanMeasurement2();
    inputs.endEffectorVelocity = endEffectorVelocity.getValueAsDouble();
    inputs.endEffectorAppliedVolts = endEffectorAppliedVolts.getValueAsDouble();
    inputs.endEffectorCurrentAmps = endEffectorCurrent.getValueAsDouble();
    inputs.endEffectorTemperature = endEffectorTemperature.getValueAsDouble();
    inputs.endEffectorPosition = endEffectorPosition.getValueAsDouble();
  }

  @Override
  public void setEndEffectorVelocity(double velocity) {
    endEffectorMotor.setControl(
        endEffectorVelocityRequest.withVelocity(velocity * EndEffectorSimConstants.GEARING));
  }

  @Override
  public void setEndEffectorSpeed(double speed) {
    endEffectorMotor.set(speed);
  }

  @Override
  public int getLaserCanMeasurement1() {
    Measurement m1 = LcEffector1.getMeasurement();
    if (m1 == null) {
      return -1;
    }
    if (m1.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return m1.distance_mm;
    } else {
      if (m1.status == LaserCan.LASERCAN_STATUS_WEAK_SIGNAL) {
        return 200;
      }
      return -1;
    }
  }

  @Override
  public int getLaserCanMeasurement2() {
    Measurement m2 = LcEffector2.getMeasurement();
    if (m2 == null) {
      return -1;
    }
    if (m2.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return m2.distance_mm;
    } else {
      if (m2.status == LaserCan.LASERCAN_STATUS_WEAK_SIGNAL) {
        return 200;
      }
      return -1;
    }
  }

  @Override
  public void endEffectorOpenLoop(Voltage voltage) {
    System.out.println(voltage);
    endEffectorMotor.setControl(endEffectorOpenLoop.withOutput(voltage));
  }
}

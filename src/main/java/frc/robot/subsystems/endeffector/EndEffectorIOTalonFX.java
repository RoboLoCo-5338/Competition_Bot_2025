package frc.robot.subsystems.endeffector;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.generated.TunerConstants;

public class EndEffectorIOTalonFX implements EndEffectorIO {

  private final TalonFX endEffectorMotor;

  private final VelocityVoltage endEffectorVelocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<AngularVelocity> endEffectorVelocity;
  private final StatusSignal<Voltage> endEffectorAppliedVolts;
  private final StatusSignal<Current> endEffectorCurrent;

  private final Debouncer effectorDebouncer = new Debouncer(0.5);

  private final LaserCan LcEffector1;
  private final LaserCan LcEffector2; // why is it screaming at me here

  public EndEffectorIOTalonFX() {

    endEffectorMotor =
        new TalonFX(EndEffectorConstants.EFFECTORID, TunerConstants.DrivetrainConstants.CANBusName);

    endEffectorVelocity = endEffectorMotor.getVelocity();
    endEffectorAppliedVolts = endEffectorMotor.getMotorVoltage();
    endEffectorCurrent = endEffectorMotor.getStatorCurrent();

    endEffectorMotor.getConfigurator().apply(getEndEffectorConfiguration());

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, endEffectorVelocity, endEffectorAppliedVolts, endEffectorCurrent));

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
    // TODO change these values
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // TODO requres certain canges - remember
    // IF NOT CHANGED WILL NOT WORK
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
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

  public SparkFlexConfig getEffectorConfig() {
    SparkFlexConfig effectorConfig = new SparkFlexConfig();

    effectorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(EndEffectorConstants.EFFECTOR_CURRENT_LIMIT)
        .voltageCompensation(12.0);
    effectorConfig
        .encoder
        .positionConversionFactor(EndEffectorConstants.EFFECTOR_ENCODER_POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(EndEffectorConstants.EFFECTOR_ENCODER_VELOCITY_CONVERSION_FACTOR)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);
    effectorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            EndEffectorConstants.EFFECTOR_KP, EndEffectorConstants.EFFECTOR_KI,
            EndEffectorConstants.EFFECTOR_KD, EndEffectorConstants.EFFECTOR_KFF);
    effectorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(10)
        .appliedOutputPeriodMs(10)
        .busVoltagePeriodMs(10)
        .outputCurrentPeriodMs(10);

    return effectorConfig;
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    var motor1Status =
        BaseStatusSignal.refreshAll(
            endEffectorVelocity, endEffectorCurrent, endEffectorAppliedVolts);

    inputs.endEffectorConnected = effectorDebouncer.calculate(motor1Status.isOK());

    inputs.endEffectorVelocity = Units.rotationsToRadians(endEffectorVelocity.getValueAsDouble());
    inputs.endEffectorAppliedVolts = endEffectorAppliedVolts.getValueAsDouble();
    inputs.endEffectorCurrentAmps = endEffectorCurrent.getValueAsDouble();

    Measurement m = LcEffector1.getMeasurement();
    if (m != null && m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.endEffectorDistance1 = m.distance_mm;
    } else {
      inputs.endEffectorDistance1 = -1;
    }
    m = LcEffector2.getMeasurement();
    if (m != null && m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.endEffectorDistance2 = m.distance_mm;
    } else {
      inputs.endEffectorDistance2 = -1;
    }
  }

  @Override
  public void setEndEffectorVelocity(double velocity) {
    endEffectorMotor.setControl(endEffectorVelocityRequest.withVelocity(velocity));
  }
}

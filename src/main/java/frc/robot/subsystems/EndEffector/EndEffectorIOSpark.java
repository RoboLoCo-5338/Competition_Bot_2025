package frc.robot.subsystems.EndEffector;

import static frc.robot.util.SparkUtil.*;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants.EndEffectorConstants;
import java.util.function.DoubleSupplier;

public class EndEffectorIOSpark implements EndEffectorIO {

  private final SparkFlex effectorMotor;

  private final RelativeEncoder effectorEncoder;

  private final SparkClosedLoopController effectorClosedLoopController;

  private final Debouncer effectorDebouncer = new Debouncer(0.5);

  private final LaserCan LcEffector1;
  private final LaserCan LcEffector2; // why is it screaming at me here

  public EndEffectorIOSpark() {

    effectorMotor = new SparkFlex(EndEffectorConstants.EFFECTORID, MotorType.kBrushless);
    effectorEncoder = effectorMotor.getEncoder();
    effectorClosedLoopController = effectorMotor.getClosedLoopController();

    tryUntilOk(
        effectorMotor,
        5,
        () ->
            effectorMotor.configure(
                getEffectorConfig(),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

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
    sparkStickyFault = false;
    ifOk(
        effectorMotor, effectorEncoder::getVelocity, (value) -> inputs.endEffectorVelocity = value);
    ifOk(
        effectorMotor,
        new DoubleSupplier[] {effectorMotor::getAppliedOutput, effectorMotor::getBusVoltage},
        (values) -> inputs.endEffectorAppliedVolts = values[0] * values[1]);
    ifOk(
        effectorMotor,
        effectorMotor::getOutputCurrent,
        (value) -> inputs.endEffectorCurrentAmps = value);
    inputs.endEffectorConnected = effectorDebouncer.calculate(!sparkStickyFault);

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
    effectorClosedLoopController.setReference(velocity, ControlType.kVelocity);
  }
}

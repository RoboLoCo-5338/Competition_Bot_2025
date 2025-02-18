package frc.robot.subsystems.groundintake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants.GroundIntakeConstants;
import java.util.function.DoubleSupplier;

public class GroundIntakeIOSpark implements GroundIntakeIO {

  private final SparkFlex armMotor;

  private final AbsoluteEncoder armEncoder;

  private final SparkClosedLoopController armController;

  private final SparkFlex intakeMotor;

  private final RelativeEncoder intakEncoder;

  private final SparkClosedLoopController intakeController;

  private final Debouncer armConnectedDebounce = new Debouncer(0.5);

  private final Debouncer intakeConnectedDebounce = new Debouncer(0.5);

  public GroundIntakeIOSpark() {

    armMotor = new SparkFlex(GroundIntakeConstants.ARM_CANID, MotorType.kBrushless);
    armEncoder = armMotor.getAbsoluteEncoder();
    armController = armMotor.getClosedLoopController();

    intakeMotor = new SparkFlex(GroundIntakeConstants.INTAKE_CANID, MotorType.kBrushless);
    intakEncoder = intakeMotor.getEncoder();
    intakeController = intakeMotor.getClosedLoopController();

    tryUntilOk(
        armMotor,
        5,
        () ->
            armMotor.configure(
                getArmConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                getIntakeConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public SparkFlexConfig getIntakeConfig() {
    SparkFlexConfig intakeConfig = new SparkFlexConfig();

    intakeConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(GroundIntakeConstants.INTAKE_CURRENT_LIMIT)
        .voltageCompensation(12.0);
    intakeConfig
        .encoder
        .positionConversionFactor(GroundIntakeConstants.INTAKE_ENCODER_POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(GroundIntakeConstants.INTAKE_ENCODER_VELOCITY_CONVERSION_FACTOR)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);
    intakeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            GroundIntakeConstants.INTAKE_KP, GroundIntakeConstants.INTAKE_KI,
            GroundIntakeConstants.INTAKE_KD, GroundIntakeConstants.INTAKE_KFF);
    intakeConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(10)
        .appliedOutputPeriodMs(10)
        .busVoltagePeriodMs(10)
        .outputCurrentPeriodMs(10);
    return intakeConfig;
  }

  public SparkFlexConfig getArmConfig() {
    SparkFlexConfig armConfig = new SparkFlexConfig();
    armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(GroundIntakeConstants.ARM_CURRENT_LIMIT)
        .voltageCompensation(12);

    armConfig
        .absoluteEncoder
        // TODO CHECK THIS
        .inverted(false)
        .positionConversionFactor(GroundIntakeConstants.ARM_ENCODER_POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(GroundIntakeConstants.ARM_ENCODER_VELOCITY_CONVERSION_FACTOR);

    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(false)
        .pidf(
            GroundIntakeConstants.ARM_KP,
            GroundIntakeConstants.ARM_KI,
            GroundIntakeConstants.ARM_KD,
            GroundIntakeConstants.ARM_KFF);

    armConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) 10.0)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    return armConfig;
  }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {

    sparkStickyFault = false;
    ifOk(armMotor, armEncoder::getPosition, (value) -> inputs.armPositionRad = value);
    ifOk(armMotor, armEncoder::getVelocity, (value) -> inputs.armVelocityRadPerSec = value);
    ifOk(
        armMotor,
        new DoubleSupplier[] {armMotor::getAppliedOutput, armMotor::getBusVoltage},
        (values) -> inputs.armAppliedVolts = values[0] * values[1]);
    ifOk(armMotor, armMotor::getOutputCurrent, (value) -> inputs.armCurrentAmps = value);

    inputs.armMotorConnected = armConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(intakeMotor, intakEncoder::getVelocity, (value) -> inputs.intakeVelocityRadPerSec = value);
    ifOk(
        intakeMotor,
        new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
        (values) -> inputs.intakeAppliedVolts = values[0] * values[1]);
    ifOk(intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.intakeCurrentAmps = value);

    inputs.intakeMotorConnected = intakeConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setArmVelocity(double velocityRadPerSec) {
    double ffvolts =
        GroundIntakeConstants.ARM_KS * Math.signum(velocityRadPerSec)
            + GroundIntakeConstants.ARM_KV * velocityRadPerSec;
    armController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffvolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setArmPosition(double position) {
    armController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void setIntakeVelocity(double velocityRadPerSec) {
    double ffvolts =
        GroundIntakeConstants.INTAKE_KS * Math.signum(velocityRadPerSec)
            + GroundIntakeConstants.INTAKE_KV * velocityRadPerSec;
    intakeController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffvolts,
        ArbFFUnits.kVoltage);
  }
}

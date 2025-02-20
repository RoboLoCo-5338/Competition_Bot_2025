package frc.robot.subsystems.groundintake;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.GroundIntakeConstants;

public interface GroundIntakeIO {
  SparkFlex armMotor = new SparkFlex(GroundIntakeConstants.ArmConstants.ARM_CANID, MotorType.kBrushless);
  SparkFlex intakeMotor = new SparkFlex(GroundIntakeConstants.INTAKE_CANID, MotorType.kBrushless);
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

  public default SparkFlexConfig getIntakeConfig() {
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

  public default SparkFlexConfig getArmConfig() {
    SparkFlexConfig armConfig = new SparkFlexConfig();
    armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(GroundIntakeConstants.ArmConstants.ARM_CURRENT_LIMIT)
        .voltageCompensation(12);

    armConfig
        .absoluteEncoder
        // TODO CHECK THIS
        .inverted(false)
        .positionConversionFactor(GroundIntakeConstants.ArmConstants.ARM_ENCODER_POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(GroundIntakeConstants.ArmConstants.ARM_ENCODER_VELOCITY_CONVERSION_FACTOR);

    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(false)
        .pidf(
            GroundIntakeConstants.ArmConstants.ARM_KP,
            GroundIntakeConstants.ArmConstants.ARM_KI,
            GroundIntakeConstants.ArmConstants.ARM_KD,
            GroundIntakeConstants.ArmConstants.ARM_KFF);

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

}

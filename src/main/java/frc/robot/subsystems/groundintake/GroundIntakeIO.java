package frc.robot.subsystems.groundintake;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.GroundIntakeConstants;
import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
  SparkFlex armMotor =
      new SparkFlex(GroundIntakeConstants.ArmConstants.ARM_CANID, MotorType.kBrushless);
  SparkFlex intakeMotor =
      new SparkFlex(GroundIntakeConstants.IntakeConstants.INTAKE_CANID, MotorType.kBrushless);
  SparkClosedLoopController intakeController = intakeMotor.getClosedLoopController();
  SparkClosedLoopController armController = armMotor.getClosedLoopController();

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

  public default void setIntakeSpeed(double speed) {}

  public default SparkFlexConfig getIntakeConfig() {
    SparkFlexConfig intakeConfig = new SparkFlexConfig();

    intakeConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(GroundIntakeConstants.IntakeConstants.INTAKE_CURRENT_LIMIT)
        .voltageCompensation(12.0);
    intakeConfig
        .encoder
        .positionConversionFactor(1 / GroundIntakeConstants.IntakeConstants.GEARING)
        .velocityConversionFactor(1 / GroundIntakeConstants.IntakeConstants.GEARING)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);
    intakeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            GroundIntakeConstants.IntakeConstants.INTAKE_KP,
                GroundIntakeConstants.IntakeConstants.INTAKE_KI,
            GroundIntakeConstants.IntakeConstants.INTAKE_KD,
                GroundIntakeConstants.IntakeConstants.INTAKE_KFF);
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
        .inverted(true)
        .positionConversionFactor(1 / GroundIntakeConstants.ArmConstants.GEARING)
        .velocityConversionFactor(1 / GroundIntakeConstants.ArmConstants.GEARING);

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

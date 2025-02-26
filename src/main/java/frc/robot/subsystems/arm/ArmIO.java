package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.ArmConstants;

public interface ArmIO {

  SparkFlex armMotor = new SparkFlex(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
  SparkClosedLoopController armClosedLoopController = armMotor.getClosedLoopController();

  @AutoLog
  public static class ArmIOInputs {
    public double armPosition = 0.0;
    public double armVelocity = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrent = 0.0;
    public boolean armConnected = false;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmPosition(double position) {}

  public default void setArmVelocity(double velocity) {}

  public default SparkFlexConfig getArmConfig() {
    SparkFlexConfig armConfig = new SparkFlexConfig();

    armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT)
        .voltageCompensation(12.0);
    armConfig
        .absoluteEncoder
        // TODO CHECK THIS
        .inverted(false)
        .positionConversionFactor(ArmConstants.ARM_ENCODER_POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(ArmConstants.ARM_ENCODER_VELOCITY_CONVERSION_FACTOR);
    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pidf(
            ArmConstants.ARM_MOTOR_KP, ArmConstants.ARM_MOTOR_KI,
            ArmConstants.ARM_MOTOR_KD, ArmConstants.ARM_MOTOR_KFF);
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

package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.AutoLog;

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

  /**
   * Updates the set of loggable inputs for the arm subsystem. This function updates the following
   * inputs:
   *
   * <ul>
   *   <li>{@code armConnected}: Whether the arm motor is connected
   *   <li>{@code armPosition}: The position of the arm motor in radians
   *   <li>{@code armVelocity}: The velocity of the arm motor in radians per second
   *   <li>{@code armAppliedVolts}: The voltage applied to the arm motor in volts
   *   <li>{@code armCurrent}: The current drawn by the arm motor in amps
   * </ul>
   */
  public default void updateInputs(ArmIOInputs inputs) {}

  /**
   * Sets the position of the arm in radians. This method is "fire-and-forget" in the sense that it
   * will not block or wait for the arm to reach the desired position. If you want to verify that
   * the arm has reached the desired position, you must poll the {@link ArmIOInputs#armPosition}
   * loggable input. This method is intended to be used with position control, not velocity control.
   * If you want to set the velocity of the arm, use {@link #setArmVelocity(double)} instead.
   *
   * @param position the desired position in radians
   */
  public default void setArmPosition(double position) {}

  /**
   * Sets the velocity of the arm in radians per second. This method is "fire-and-forget" in the
   * sense that it will not block or wait for the arm to reach the desired velocity. If you want to
   * verify that the arm has reached the desired velocity, you must poll the velocity using {@link
   * #updateInputs(ArmIOInputs)}.
   *
   * @param velocityRadPerSec The velocity of the arm in radians per second.
   */
  public default void setArmVelocity(double velocity) {}

  /**
   * Gets the configuration used for the Talon FX motor controllers of the arm subsystem.
   *
   * <p>This method returns a Talon FX configuration with the following settings:
   *
   * <ul>
   *   <li>Neutral mode: Brake
   *   <li>Gravity type: Arm cosine
   *   <li>Feedback device: Integrated sensor
   *   <li>kP: {@link ArmConstants#ARM_MOTOR_kP}
   *   <li>kI: {@link ArmConstants#ARM_MOTOR_kI}
   *   <li>kD: {@link ArmConstants#ARM_MOTOR_kD}
   *   <li>kG: {@link ArmConstants#ARM_MOTOR_kG}
   *   <li>kV: {@link ArmConstants#ARM_MOTOR_kV}
   *   <li>Current limit: 40A (CHANGE THIS VALUE OTHERWISE TORQUE MAY BE LIMITED/TOO HIGH)
   * </ul>
   *
   * <p>These values may need to be changed based on the actual robot hardware and the desired
   * behavior of the elevator.
   *
   * @return the configuration used for the Talon FX motor controllers of the arm subsystem
   */
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
        .positionConversionFactor(1/ArmConstants.GEARING)
        .velocityConversionFactor(1/ArmConstants.GEARING);
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

  public default void updatePID() {
    SparkFlexConfig armConfig = new SparkFlexConfig();
    armConfig.closedLoop.pidf(
        ArmConstants.ARM_MOTOR_KP, ArmConstants.ARM_MOTOR_KI,
        ArmConstants.ARM_MOTOR_KD, ArmConstants.ARM_MOTOR_KFF);
    armMotor.configure(
        armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}

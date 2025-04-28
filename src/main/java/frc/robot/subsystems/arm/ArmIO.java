package frc.robot.subsystems.arm;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.subsystems.arm.ArmConstants.ArmSimConstants;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  //new spark flex motor
  SparkFlex armMotor = new SparkFlex(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
  SparkClosedLoopController armClosedLoopController = armMotor.getClosedLoopController();
  
  //autologger
  @AutoLog
  public static class ArmIOInputs {
    public double armPosition = 0.0;
    public double armVelocity = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrent = 0.0;
    public boolean armConnected = false;
    public double armTemperature = 0.0;
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
        //sets it so that arm stays still when idle instead of falling
        .idleMode(IdleMode.kBrake)
        //inverted
        .inverted(true)
        //current limit for brushless motors
        .smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT)
        //when getting a percentage of voltage, it bases it off this value
        .voltageCompensation(12.0);

    //absolute encoder config
    armConfig
        .absoluteEncoder
        //inverted
        .inverted(true)
        //gearing stuff
        .positionConversionFactor(1 / ArmSimConstants.GEARING)
        .velocityConversionFactor(2.0 / ArmSimConstants.GEARING);

    //feedback closed loop (pid)
    armConfig
        .closedLoop
        //uses the absolute encoder as the feedback sensor to measure error
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        //no position wrapping b/c only want arm to go the direction we tell it to
        .positionWrappingEnabled(false)
        //for target position
        .pid(
            ArmConstants.ARM_MOTOR_POSITION_KP, ArmConstants.ARM_MOTOR_POSITION_KI,
            ArmConstants.ARM_MOTOR_POSITION_KD, ClosedLoopSlot.kSlot0)
        //for target velocity
        .pid(
            ArmConstants.ARM_MOTOR_VELOCITY_KP, ArmConstants.ARM_MOTOR_VELOCITY_KI,
            ArmConstants.ARM_MOTOR_VELOCITY_KD, ClosedLoopSlot.kSlot1);
    armConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) 10.0)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    // added 3/6
    armConfig.softLimit.reverseSoftLimitEnabled(true);
    //soft limit so arm shouldn't go below 0.43
    armConfig.softLimit.reverseSoftLimit(0.43);

    return armConfig;
  }

  public default double getArmPosition(ArmIOInputs inputs) {
    //placeholder...
    return 0.0;
  }
  //cleaner version imo:
  //public double getArmPosition (ArmIOInputs inputs);
}

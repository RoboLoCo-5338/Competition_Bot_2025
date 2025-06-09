package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.*;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.arm.ArmConstants.ArmSimConstants;
import java.util.function.DoubleSupplier;

public class ArmIOSpark extends ArmIO {

  private final AbsoluteEncoder armEncoder;

  private final Debouncer armConnectedDebouncer = new Debouncer(0.5);

  private ArmFeedforward feedforward;

  SparkFlex armMotor = new SparkFlex(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
  SparkClosedLoopController armClosedLoopController = armMotor.getClosedLoopController();

  public ArmIOSpark() {
    armEncoder = armMotor.getAbsoluteEncoder();

    feedforward =
        new ArmFeedforward(
            ArmConstants.ARM_MOTOR_KS, ArmConstants.ARM_MOTOR_KG, ArmConstants.ARM_MOTOR_KV);

    tryUntilOk(
        armMotor,
        5,
        () ->
            armMotor.configure(
                getArmConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

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
  public SparkFlexConfig getArmConfig() {
    SparkFlexConfig armConfig = new SparkFlexConfig();

    armConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT)
        .voltageCompensation(12.0);
    armConfig
        .absoluteEncoder
        .zeroCentered(true)
        .inverted(true)
        .positionConversionFactor(1 / ArmSimConstants.GEARING)
        .velocityConversionFactor(
            2.0 / ArmSimConstants.GEARING); // TODO: figure out why we need this to be 2.0
    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(false)
        .pid(
            ArmConstants.ARM_MOTOR_POSITION_KP, ArmConstants.ARM_MOTOR_POSITION_KI,
            ArmConstants.ARM_MOTOR_POSITION_KD, ClosedLoopSlot.kSlot0)
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

    // // added 3/6
    // armConfig.softLimit.reverseSoftLimitEnabled(true);
    // armConfig.softLimit.reverseSoftLimit(ArmConstants.SOFT_LIMIT);

    return armConfig;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sparkStickyFault = false;
    // SmartDashboard.putNumber("ArmPosition Before", inputs.armPosition);
    ifOk(armMotor, armEncoder::getPosition, (value) -> inputs.position = value);
    // SmartDashboard.putNumber("ArmPosition After", inputs.armPosition);
    ifOk(armMotor, armEncoder::getVelocity, (value) -> inputs.velocity = value);
    ifOk(
        armMotor,
        new DoubleSupplier[] {armMotor::getAppliedOutput, armMotor::getBusVoltage},
        (values) -> inputs.armAppliedVolts = values[0] * values[1]);
    ifOk(armMotor, armMotor::getOutputCurrent, (value) -> inputs.armCurrent = value);
    ifOk(armMotor, armMotor::getMotorTemperature, (value) -> inputs.armTemperature = value);

    inputs.armConnected = armConnectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setArmPosition(double position) {

    armClosedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setArmVelocity(double velocityRadPerSec) {

    double ffvolts =
        feedforward.calculate((armEncoder.getPosition()) * 2 * Math.PI, velocityRadPerSec);

    armClosedLoopController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot1,
        ffvolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public double getArmPosition(ArmIOInputs inputs) {
    SmartDashboard.putNumber("Arm Positoin in method", inputs.position);
    return inputs.position;
  }

  @Override
  public void openLoop(Voltage voltage) {
    armMotor.setVoltage(voltage);
  }

  @Override
  public void close(){
    armMotor.close();
  }
}

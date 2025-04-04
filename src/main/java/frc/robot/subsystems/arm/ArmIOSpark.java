package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import java.util.function.DoubleSupplier;

public class ArmIOSpark implements ArmIO {

  private final AbsoluteEncoder armEncoder;

  private final Debouncer armConnectedDebouncer = new Debouncer(0.5);
  public ArmFeedforward feedforward;

  public ArmIOSpark() {
    armEncoder = armMotor.getAbsoluteEncoder();

    feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    tryUntilOk(
        armMotor,
        5,
        () ->
            armMotor.configure(
                getArmConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sparkStickyFault = false;
    // SmartDashboard.putNumber("ArmPosition Before", inputs.armPosition);
    ifOk(armMotor, armEncoder::getPosition, (value) -> inputs.armPosition = value);
    // SmartDashboard.putNumber("ArmPosition After", inputs.armPosition);
    ifOk(
        armMotor,
        () -> Units.rotationsPerMinuteToRadiansPerSecond(armEncoder.getVelocity() * 2),
        (value) -> inputs.armVelocity = value);
    ifOk(
        armMotor,
        new DoubleSupplier[] {armMotor::getAppliedOutput, armMotor::getBusVoltage},
        (values) -> inputs.armAppliedVolts = values[0] * values[1]);
    ifOk(armMotor, armMotor::getOutputCurrent, (value) -> inputs.armCurrent = value);

    inputs.armConnected = armConnectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setArmPosition(double position) {

    armClosedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setArmVelocity(double velocityRadPerSec) {
    // guys idk what this offset is, we need to measure it
    double ffvolts =
        feedforward.calculate((armEncoder.getPosition() - 0.705) * 2 * Math.PI, velocityRadPerSec);
    SmartDashboard.putNumber("ffvolts", ffvolts);
    SmartDashboard.putNumber("adjusted arm position", (armEncoder.getPosition() - 0.705));
    armClosedLoopController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot1,
        ffvolts,
        ArbFFUnits.kVoltage);
    // armMotor.set(velocityRadPerSec);
  }

  @Override
  public double getArmPosition(ArmIOInputs inputs) {
    SmartDashboard.putNumber("Arm Positoin in method", inputs.armPosition);
    return inputs.armPosition;
  }

  @Override
  public void updatePID() {
    SparkFlexConfig armConfig = new SparkFlexConfig();
    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(false)
        .pid(
            ArmConstants.positionkP, ArmConstants.positionkI,
            ArmConstants.positionkD, ClosedLoopSlot.kSlot0)
        .pid(
            ArmConstants.velocitykP,
            ArmConstants.velocitykI,
            ArmConstants.velocitykD,
            ClosedLoopSlot.kSlot1);
    feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    armMotor.configure(
        armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}

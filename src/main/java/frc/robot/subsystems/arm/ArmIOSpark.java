package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;

public class ArmIOSpark implements ArmIO {

  private final AbsoluteEncoder armEncoder;

  private final Debouncer armConnectedDebouncer = new Debouncer(0.5);

  private ArmFeedforward feedforward;

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

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sparkStickyFault = false;
    // SmartDashboard.putNumber("ArmPosition Before", inputs.armPosition);
    ifOk(armMotor, armEncoder::getPosition, (value) -> inputs.armPosition = value);
    // SmartDashboard.putNumber("ArmPosition After", inputs.armPosition);
    ifOk(armMotor, armEncoder::getVelocity, (value) -> inputs.armVelocity = value);
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

    double ffvolts = feedforward.calculate(velocityRadPerSec, velocityRadPerSec);

    armClosedLoopController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot1,
        ffvolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public double getArmPosition(ArmIOInputs inputs) {
    SmartDashboard.putNumber("Arm Positoin in method", inputs.armPosition);
    return inputs.armPosition;
  }
}

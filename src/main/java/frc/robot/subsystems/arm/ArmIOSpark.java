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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import java.util.function.DoubleSupplier;

public class ArmIOSpark implements ArmIO {

  private final AbsoluteEncoder armEncoder;

  private final Debouncer armConnectedDebouncer = new Debouncer(0.5);
  ArmFeedforward feedforward;

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
    ifOk(armMotor, armEncoder::getVelocity, (value) -> inputs.armVelocity = value);
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
    double ffvolts = feedforward.calculate(armEncoder.getPosition() - 0.34, velocityRadPerSec);
    armClosedLoopController.setReference(
        velocityRadPerSec,
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
}

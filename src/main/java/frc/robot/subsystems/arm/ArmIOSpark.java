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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;
import java.util.function.DoubleSupplier;

public class ArmIOSpark implements ArmIO {

  private final AbsoluteEncoder armEncoder;

  private final Debouncer armConnectedDebouncer = new Debouncer(0.5);

  public ArmIOSpark() {
    armEncoder = armMotor.getAbsoluteEncoder();

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
    ifOk(armMotor, armEncoder::getPosition, (value) -> inputs.armPosition = value);
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
    armClosedLoopController.setReference(Units.radiansToRotations(position), ControlType.kPosition);
  }

  @Override
  public void setArmVelocity(double velocityRadPerSec) {
    double ffvolts =
        ArmConstants.ARM_MOTOR_KS * Math.signum(velocityRadPerSec)
            + ArmConstants.ARM_MOTOR_KV * velocityRadPerSec;
    armClosedLoopController.setReference(
        Units.radiansToRotations(velocityRadPerSec),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffvolts,
        ArbFFUnits.kVoltage);
  }
}

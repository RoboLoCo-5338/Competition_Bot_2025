package frc.robot.subsystems.groundintake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class GroundIntakeIOSpark implements GroundIntakeIO {

  private final AbsoluteEncoder armEncoder;

  private final RelativeEncoder intakEncoder;

  private final Debouncer armConnectedDebounce = new Debouncer(0.5);

  private final Debouncer intakeConnectedDebounce = new Debouncer(0.5);

  public GroundIntakeIOSpark() {
    armEncoder = armMotor.getAbsoluteEncoder();

    intakEncoder = intakeMotor.getEncoder();

    tryUntilOk(
        armMotor,
        5,
        () ->
            armMotor.configure(
                getArmConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                getIntakeConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {

    sparkStickyFault = false;
    ifOk(armMotor, armEncoder::getPosition, (value) -> inputs.armPositionRad = value);
    ifOk(armMotor, armEncoder::getVelocity, (value) -> inputs.armVelocityRadPerSec = value);
    ifOk(
        armMotor,
        new DoubleSupplier[] {armMotor::getAppliedOutput, armMotor::getBusVoltage},
        (values) -> inputs.armAppliedVolts = values[0] * values[1]);
    ifOk(armMotor, armMotor::getOutputCurrent, (value) -> inputs.armCurrentAmps = value);

    inputs.armMotorConnected = armConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(intakeMotor, intakEncoder::getVelocity, (value) -> inputs.intakeVelocityRadPerSec = value);
    ifOk(
        intakeMotor,
        new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
        (values) -> inputs.intakeAppliedVolts = values[0] * values[1]);
    ifOk(intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.intakeCurrentAmps = value);

    inputs.intakeMotorConnected = intakeConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setArmVelocity(double velocityRadPerSec) {
    double ffvolts =
        GroundIntakeConstants.ArmConstants.ARM_KS * Math.signum(velocityRadPerSec)
            + GroundIntakeConstants.ArmConstants.ARM_KV * velocityRadPerSec;
    armController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffvolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setArmPosition(double position) {
    armController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void setIntakeVelocity(double velocityRadPerSec) {
    double ffvolts =
        GroundIntakeConstants.IntakeConstants.INTAKE_KS * Math.signum(velocityRadPerSec)
            + GroundIntakeConstants.IntakeConstants.INTAKE_KV * velocityRadPerSec;

    intakeController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffvolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }
}

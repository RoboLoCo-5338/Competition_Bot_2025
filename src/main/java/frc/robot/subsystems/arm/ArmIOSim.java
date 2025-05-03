package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.SimMechanism;
import frc.robot.subsystems.arm.ArmConstants.ArmSimConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class ArmIOSim extends SimMechanism implements ArmIO {

  DCMotor armGearBox = DCMotor.getNeoVortex(1);
  SingleJointedArmSim armPhysicsSim =
      new SingleJointedArmSim(
          armGearBox,
          ArmSimConstants.GEARING,
          ArmSimConstants.MOI,
          ArmSimConstants.LENGTH,
          ArmSimConstants.MIN_ANGLE,
          ArmSimConstants.MAX_ANGLE,
          true,
          ArmSimConstants.STARTING_ANGLE);
  SparkFlexSim armSim;
  SparkAbsoluteEncoderSim armEncoderSim;
  LoggedMechanismLigament2d armDrawn;

  public ArmIOSim(LoggedMechanismLigament2d endEffector) {
    super();
    SparkFlexConfig c = getArmConfig();
    c.softLimit.reverseSoftLimit(-0.3);
    // Soft limit has weird behavior in sim currently
    c.softLimit.reverseSoftLimitEnabled(false);
    armMotor.configure(c, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armSim = new SparkFlexSim(armMotor, armGearBox);
    armEncoderSim = armSim.getAbsoluteEncoderSim();
    armDrawn =
        endEffector
            .append(new LoggedMechanismLigament2d(null, ArmSimConstants.LENGTH, 0))
            .append(new LoggedMechanismLigament2d("rotator", 0, -90))
            .append(
                new LoggedMechanismLigament2d(
                    "endEffectorArm",
                    ArmSimConstants.LENGTH,
                    Units.radiansToDegrees(ArmSimConstants.STARTING_ANGLE),
                    10,
                    new Color8Bit(Color.kRed)));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    armPhysicsSim.setInputVoltage(armSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    armPhysicsSim.update(0.02);
    armSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            armPhysicsSim.getVelocityRadPerSec()),
        RobotController.getBatteryVoltage(),
        0.02);
    inputs.armConnected = true;
    inputs.armPosition =
        Units.radiansToRotations(armPhysicsSim.getAngleRads()) + ArmSimConstants.SIM_OFFSET;
    inputs.armVelocity =
        Units.radiansPerSecondToRotationsPerMinute(armPhysicsSim.getVelocityRadPerSec());
    ifOk(
        armMotor,
        new DoubleSupplier[] {armMotor::getAppliedOutput, armMotor::getBusVoltage},
        (values) -> inputs.armAppliedVolts = values[0] * values[1]);
    inputs.armCurrent = armPhysicsSim.getCurrentDrawAmps();

    armDrawn.setAngle(Units.radiansToDegrees(armPhysicsSim.getAngleRads()));
  }

  @Override
  public double[] getCurrents() {
    return new double[] {armPhysicsSim.getCurrentDrawAmps()};
  }

  @Override
  public void setArmPosition(double position) {
    double ffvolts =
        feedforward.calculate(
            Units.rotationsToRadians(armEncoderSim.getPosition() - ArmSimConstants.SIM_OFFSET),
            Units.rotationsPerMinuteToRadiansPerSecond(armEncoderSim.getVelocity()));
    armClosedLoopController.setReference(
        position - ArmSimConstants.SIM_OFFSET,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ffvolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public double getArmPosition(ArmIOInputs inputs) {
    return inputs.armPosition;
  }

  @Override
  public void setArmVelocity(double velocityRadPerSec) {
    double ffvolts =
        feedforward.calculate(
            Units.rotationsToRadians(armEncoderSim.getPosition() - ArmSimConstants.SIM_OFFSET),
            velocityRadPerSec);
    armClosedLoopController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot1,
        ffvolts,
        ArbFFUnits.kVoltage);
  }
}

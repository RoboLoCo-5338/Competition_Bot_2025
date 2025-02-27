package frc.robot.subsystems.groundintake;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.subsystems.SimMechanism;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class GroundIntakeIOSim extends SimMechanism implements GroundIntakeIO {
  DCMotor armGearBox = DCMotor.getNeoVortex(1);
  SparkFlexSim armSim;
  SparkAbsoluteEncoderSim armEncoderSim;
  SingleJointedArmSim armPhysicsSim =
      new SingleJointedArmSim(
          armGearBox,
          GroundIntakeConstants.ArmConstants.GEARING,
          GroundIntakeConstants.ArmConstants.MOI + 1,
          GroundIntakeConstants.ArmConstants.LENGTH,
          GroundIntakeConstants.ArmConstants.MIN_ANGLE,
          GroundIntakeConstants.ArmConstants.MAX_ANGLE,
          false,
          GroundIntakeConstants.ArmConstants.STARTING_ANGLE);

  @AutoLogOutput(key = "Arm/Mechanism")
  LoggedMechanism2d armDrawn =
      new LoggedMechanism2d(
          Units.inchesToMeters(16.5 * 2), 0); // Someone please improve this naming scheme

  LoggedMechanismRoot2d root = armDrawn.getRoot("root", 0, 0);
  LoggedMechanismLigament2d movingArm;

  DCMotor intakeGearBox = DCMotor.getNeoVortex(1);
  SparkFlexSim intakeSim;
  SparkAbsoluteEncoderSim intakeEncoderSim;
  FlywheelSim intakePhysicsSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              intakeGearBox,
              GroundIntakeConstants.IntakeConstants.MOI,
              GroundIntakeConstants.IntakeConstants.GEARING),
          armGearBox);

  public GroundIntakeIOSim() {
    super();
    armMotor.configure(
        getArmConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armSim = new SparkFlexSim(armMotor, armGearBox);
    armEncoderSim = new SparkAbsoluteEncoderSim(armMotor);

    intakeMotor.configure(
        getIntakeConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeSim = new SparkFlexSim(intakeMotor, intakeGearBox);
    intakeEncoderSim = new SparkAbsoluteEncoderSim(armMotor);
    movingArm =
        root.append(
                new LoggedMechanismLigament2d(
                    "base", GroundIntakeConstants.ArmConstants.ARM_BASE_HEIGHT, 90))
            .append(new LoggedMechanismLigament2d("rotator", 0, -90))
            .append(
                new LoggedMechanismLigament2d(
                    "arm",
                    GroundIntakeConstants.ArmConstants.LENGTH,
                    GroundIntakeConstants.ArmConstants.STARTING_ANGLE));
  }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {
    armPhysicsSim.setInputVoltage(armSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    intakePhysicsSim.setInput(intakeSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    armPhysicsSim.update(0.02);
    intakePhysicsSim.update(0.02);

    armSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            armPhysicsSim.getVelocityRadPerSec() * GroundIntakeConstants.ArmConstants.GEARING),
        RobotController.getBatteryVoltage(),
        0.02);
    intakeSim.iterate(
        intakePhysicsSim.getAngularVelocityRPM() * GroundIntakeConstants.IntakeConstants.GEARING,
        RobotController.getBatteryVoltage(),
        0.02);

    inputs.armMotorConnected = true;
    inputs.armPositionRad = armPhysicsSim.getAngleRads();
    inputs.armVelocityRadPerSec = armPhysicsSim.getVelocityRadPerSec();
    ifOk(
        armMotor,
        new DoubleSupplier[] {armMotor::getAppliedOutput, armMotor::getBusVoltage},
        (values) -> inputs.armAppliedVolts = values[0] * values[1]);
    inputs.armCurrentAmps = armPhysicsSim.getCurrentDrawAmps();

    movingArm.setAngle(Units.radiansToDegrees(armPhysicsSim.getAngleRads()));

    inputs.intakeMotorConnected = true;
    inputs.intakeVelocityRadPerSec = intakePhysicsSim.getAngularVelocityRadPerSec();
    ifOk(
        intakeMotor,
        new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
        (values) -> inputs.intakeAppliedVolts = values[0] * values[1]);
    inputs.intakeCurrentAmps = intakePhysicsSim.getCurrentDrawAmps();
  }

  @Override
  public void setArmVelocity(double velocityRadPerSec) {
    double ffvolts =
        GroundIntakeConstants.ArmConstants.ARM_KS * Math.signum(velocityRadPerSec)
            + GroundIntakeConstants.ArmConstants.ARM_KV * velocityRadPerSec;
    armController.setReference(
        velocityRadPerSec * GroundIntakeConstants.ArmConstants.GEARING,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffvolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setArmPosition(double position) {
    armController.setReference(
        position * GroundIntakeConstants.ArmConstants.GEARING, ControlType.kPosition);
  }

  @Override
  public void setIntakeVelocity(double velocityRadPerSec) {
    double ffvolts =
        GroundIntakeConstants.IntakeConstants.INTAKE_KS * Math.signum(velocityRadPerSec)
            + GroundIntakeConstants.IntakeConstants.INTAKE_KV * velocityRadPerSec;
    intakeController.setReference(
        velocityRadPerSec * GroundIntakeConstants.IntakeConstants.GEARING,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffvolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public double[] getCurrents() {
    return new double[] {armSim.getMotorCurrent(), intakeSim.getMotorCurrent()};
  }
}

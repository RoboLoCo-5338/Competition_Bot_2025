package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.SimMechanism;
import frc.robot.subsystems.arm.ArmConstants.ArmSimConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class ArmIOSim extends ArmIOSpark implements SimMechanism {

  DCMotor armGearBox = DCMotor.getNeoVortex(1);
  SingleJointedArmSim armPhysicsSim =
      new SingleJointedArmSim(
          armGearBox,
          ArmSimConstants.GEARING,
          ArmSimConstants.MOI,
          ArmSimConstants.LENGTH,
          ArmSimConstants.MIN_ANGLE,
          ArmSimConstants.MAX_ANGLE,
          false,
          ArmSimConstants.STARTING_ANGLE);
  SparkFlexSim armSim;
  SparkAbsoluteEncoderSim armEncoderSim;
  LoggedMechanismLigament2d armDrawn;

  public ArmIOSim(LoggedMechanismLigament2d endEffector) {
    super();
    armSim = new SparkFlexSim(armMotor, armGearBox);
    armEncoderSim = new SparkAbsoluteEncoderSim(armMotor);
    armDrawn =
        endEffector
            .append(new LoggedMechanismLigament2d("rotator", 0, -90))
            .append(
                new LoggedMechanismLigament2d(
                    "endEffectorArm",
                    ArmSimConstants.LENGTH,
                    Units.radiansToDegrees(ArmSimConstants.STARTING_ANGLE)));
    initSimVoltage();
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

    Logger.recordOutput("Arm Position", Units.radiansToRotations(armPhysicsSim.getAngleRads()));
    Logger.recordOutput(
        "Arm Velocity", Units.radiansToRotations(armPhysicsSim.getVelocityRadPerSec()));
    ifOk(
        armMotor,
        new DoubleSupplier[] {armMotor::getAppliedOutput, armMotor::getBusVoltage},
        (values) -> Logger.recordOutput("armAppliedVolts", values[0] * values[1]));
    Logger.recordOutput("Arm Current", armPhysicsSim.getCurrentDrawAmps());

    armDrawn.setAngle(Units.radiansToDegrees(armPhysicsSim.getAngleRads()));

    super.updateInputs(inputs);
  }

  @Override
  public double[] getCurrents() {
    return new double[] {armPhysicsSim.getCurrentDrawAmps()};
  }
}

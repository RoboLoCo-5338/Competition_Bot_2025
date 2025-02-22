package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.SimMechanism;
import java.util.function.DoubleSupplier;

public class ArmIOSim extends SimMechanism implements ArmIO {

  DCMotor armGearBox = DCMotor.getNeoVortex(1);
  SparkAbsoluteEncoderSim armEncoderSim;
  SingleJointedArmSim armPhysicsSim =
      new SingleJointedArmSim(
          armGearBox,
          ArmConstants.GEARING,
          ArmConstants.MOI + 1,
          ArmConstants.LENGTH,
          ArmConstants.MIN_ANGLE,
          ArmConstants.MAX_ANGLE,
          false,
          ArmConstants.STARTING_ANGLE);
  SparkFlexSim armSim;

  public ArmIOSim() {
    super();
    armMotor.configure(
        getArmConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armSim = new SparkFlexSim(armMotor, armGearBox);
    armEncoderSim = new SparkAbsoluteEncoderSim(armMotor);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    ifOk(armMotor, armEncoderSim::getPosition, (value) -> inputs.armPosition = value);
    ifOk(armMotor, armEncoderSim::getVelocity, (value) -> inputs.armVelocity = value);
    ifOk(
        armMotor,
        new DoubleSupplier[] {armMotor::getAppliedOutput, armMotor::getBusVoltage},
        (values) -> inputs.armAppliedVolts = values[0] * values[1]);
    ifOk(armMotor, armMotor::getOutputCurrent, (value) -> inputs.armCurrent = value);
  }

  @Override
  public double[] getCurrents() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCurrents'");
  }
}

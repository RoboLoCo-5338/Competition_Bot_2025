package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmCommands;
import frc.robot.subsystems.SimMechanism;

public class ArmIOSim extends SimMechanism implements ArmIO {

  DCMotor armGearBox = DCMotor.getNeoVortex(1);
  SparkAbsoluteEncoderSim armEncoderSim;
  SingleJointedArmSim armPhysicsSim = new SingleJointedArmSim(armGearBox, ArmConstants.GEARING, ArmConstants.MOI, ArmConstants.length, ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE, false, ArmConstants.STARTING_ANGLE);
  SparkFlexSim armSim;



  public ArmIOSim() {
    super();
    
    armSim = new SparkFlexSim(armMotor, armGearBox);
    armEncoderSim = new SparkAbsoluteEncoderSim(armMotor);

  }

  @Override
  public double[] getCurrents() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCurrents'");
  }
}

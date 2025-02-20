package frc.robot.subsystems.groundintake;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.subsystems.SimMechanism;

public class GroundIntakeIOSim extends SimMechanism implements GroundIntakeIO {
    DCMotor armGearBox = DCMotor.getNeoVortex(1);
    SparkFlexSim armSim;
    SparkAbsoluteEncoderSim armEncoderSim;
    SingleJointedArmSim armPhysicsSim = new SingleJointedArmSim(armGearBox, GroundIntakeConstants.ArmConstants.GEARING, GroundIntakeConstants.ArmConstants.MOI, GroundIntakeConstants.ArmConstants.LENGTH, GroundIntakeConstants.ArmConstants.MIN_ANGLE, GroundIntakeConstants.ArmConstants.MAX_ANGLE, false, GroundIntakeConstants.ArmConstants.STARTING_ANGLE);

    DCMotor intakeGearBox = DCMotor.getNeoVortex(1);
    SparkFlexSim intakeSim;
    SparkAbsoluteEncoderSim intakeEncoderSim;
    FlywheelSim intakePhysicsSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(intakeGearBox, 0, 0), armGearBox, getCurrents());
    public GroundIntakeIOSim(){
        super();
        armMotor.configure(getArmConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armSim = new SparkFlexSim(armMotor, armGearBox);
        armEncoderSim = new SparkAbsoluteEncoderSim(armMotor);

        intakeMotor.configure(getIntakeConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeSim = new SparkFlexSim(intakeMotor, intakeGearBox);
        intakeEncoderSim = new SparkAbsoluteEncoderSim(armMotor);
    }
    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {}

    @Override
    public void setArmVelocity(double velocityRadPerSec) {}
  
    @Override
    public void setArmPosition(double position) {}
  
    @Override
    public void setIntakeVelocity(double velocityRadPerSec) {}
    @Override
    public double[] getCurrents() {
        return new double[0];
    }
}

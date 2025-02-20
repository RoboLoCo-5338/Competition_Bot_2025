package frc.robot.subsystems.groundintake;

import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

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
    FlywheelSim intakePhysicsSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(intakeGearBox, GroundIntakeConstants.IntakeConstants.MOI, GroundIntakeConstants.IntakeConstants.GEARING), armGearBox);
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
    public void updateInputs(GroundIntakeIOInputs inputs) {
        ifOk(armMotor, armEncoderSim::getPosition, (value) -> inputs.armPositionRad = value);
        ifOk(armMotor, armEncoderSim::getVelocity, (value) -> inputs.armVelocityRadPerSec = value);
        ifOk(
            armMotor,
            new DoubleSupplier[] {armMotor::getAppliedOutput, armMotor::getBusVoltage},
            (values) -> inputs.armAppliedVolts = values[0] * values[1]);
        ifOk(armMotor, armMotor::getOutputCurrent, (value) -> inputs.armCurrentAmps = value);

        ifOk(intakeMotor, intakeEncoderSim::getVelocity, (value) -> inputs.intakeVelocityRadPerSec = value);
        ifOk(
            intakeMotor,
            new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
            (values) -> inputs.intakeAppliedVolts = values[0] * values[1]);
        ifOk(intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.intakeCurrentAmps = value);
    }

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

package frc.robot.subsystems.endeffector;

import org.dyn4j.geometry.Triangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.sim.TalonFXSimState;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.SimMechanism;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EndEffectorSimConstants;

public class EndEffectorIOSim implements SimMechanism, EndEffectorIO {
  TalonFXSimState simMotor = endEffectorMotor.getSimState();
  FlywheelSim physicsSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1), EndEffectorSimConstants.MOI, EndEffectorConstants.GEARING),
          DCMotor.getKrakenX60(1));
  IntakeSimulation intakeSim;

  public EndEffectorIOSim(SwerveDriveSimulation driveSim) {
    initSimVoltage();
    endEffectorMotor.getConfigurator().apply(getEndEffectorConfiguration());
    this.intakeSim = new IntakeSimulation("Coral", driveSim, new Triangle(new Vector2(), null, null), 1);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    physicsSim.setInputVoltage(simMotor.getMotorVoltage());

    inputs.endEffectorConnected = true;
    inputs.endEffectorVelocity = Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec());
    inputs.endEffectorAppliedVolts = physicsSim.getInputVoltage();
    inputs.endEffectorCurrentAmps = physicsSim.getCurrentDrawAmps();

    physicsSim.update(0.02);

    simMotor.addRotorPosition(
        Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec())
            * 0.02
            * EndEffectorConstants.GEARING);
    simMotor.setRotorVelocity(
        Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec())
            * EndEffectorConstants.GEARING);
  }

  @Override
  public void setEndEffectorVelocity(double velocity) {
    endEffectorMotor.setControl(
        endEffectorVelocityRequest.withVelocity(velocity * EndEffectorConstants.GEARING));
  }

  @Override
  public void setEndEffectorSpeed(double speed) {
    endEffectorMotor.set(speed * EndEffectorConstants.GEARING);
  }

  @Override
  public double[] getCurrents() {
    return new double[] {physicsSim.getCurrentDrawAmps()};
  }

  @Override
  public double getEndEffectorVelocity() {
    return physicsSim.getAngularVelocityRadPerSec();
  }
}

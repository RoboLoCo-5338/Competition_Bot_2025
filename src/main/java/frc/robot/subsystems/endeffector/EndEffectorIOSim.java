package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.sim.TalonFXSimState;
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

  public EndEffectorIOSim() {
    initSimVoltage();
    endEffectorMotor.getConfigurator().apply(getEndEffectorConfiguration());
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

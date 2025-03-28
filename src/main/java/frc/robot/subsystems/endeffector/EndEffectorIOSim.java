package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.SimMechanism;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EndEffectorSimConstants;

public class EndEffectorIOSim extends SimMechanism implements EndEffectorIO {
  TalonFXSimState simMotor = endEffectorMotor.getSimState();
  FlywheelSim physicsSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1), EndEffectorSimConstants.MOI, EndEffectorSimConstants.GEARING),
          DCMotor.getKrakenX60(1));

  public EndEffectorIOSim() {
    super();
    endEffectorMotor.getConfigurator().apply(getEndEffectorConfiguration());
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    physicsSim.setInputVoltage(simMotor.getMotorVoltage());

    inputs.endEffectorConnected = true;
    inputs.endEffectorVelocity = physicsSim.getAngularVelocityRPM();
    inputs.endEffectorAppliedVolts = physicsSim.getInputVoltage();
    inputs.endEffectorCurrentAmps = physicsSim.getCurrentDrawAmps();

    physicsSim.update(0.02);

    simMotor.addRotorPosition(
        physicsSim.getAngularVelocityRadPerSec() * 0.02 * EndEffectorSimConstants.GEARING);
    simMotor.setRotorVelocity(
        physicsSim.getAngularVelocityRadPerSec() * EndEffectorSimConstants.GEARING);
  }

  @Override
  public void setEndEffectorVelocity(double velocity) {
    endEffectorMotor.setControl(
        endEffectorVelocityRequest.withVelocity(velocity * EndEffectorSimConstants.GEARING));
  }

  @Override
  public double[] getCurrents() {
    return new double[] {physicsSim.getCurrentDrawAmps()};
  }
}

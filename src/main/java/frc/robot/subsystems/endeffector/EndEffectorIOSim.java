package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.SimMechanism;

public class EndEffectorIOSim extends SimMechanism implements EndEffectorIO {
  TalonFXSimState simMotor = endEffectorMotor.getSimState();
  FlywheelSim physicsSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1), EndEffectorConstants.MOI, EndEffectorConstants.GEARING),
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
        physicsSim.getAngularVelocityRadPerSec() * 0.02 * EndEffectorConstants.GEARING);
    simMotor.setRotorVelocity(
        physicsSim.getAngularVelocityRadPerSec() * EndEffectorConstants.GEARING);
  }

  @Override
  public void setEndEffectorVelocity(double velocity) {
    endEffectorMotor.setControl(
        endEffectorVelocityRequest.withVelocity(velocity * EndEffectorConstants.GEARING));
  }

  @Override
  public double[] getCurrents() {
    return new double[] {physicsSim.getCurrentDrawAmps()};
  }
}

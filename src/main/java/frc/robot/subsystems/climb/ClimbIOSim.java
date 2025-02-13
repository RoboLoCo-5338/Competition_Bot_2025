package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ClimbConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ClimbIOSim implements ClimbIO {
  @AutoLogOutput(key = "Climb/Mechanism")

  //The logged shape and stuff of the mechanism
  LoggedMechanism2d mechanism = new LoggedMechanism2d(5, 5);
  LoggedMechanismRoot2d root = mechanism.getRoot("climb", 2.5, 0);
  LoggedMechanismLigament2d m_climbBase = root.append(new LoggedMechanismLigament2d("base", 1, 90));
  LoggedMechanismLigament2d m_climbArm =
      m_climbBase.append(new LoggedMechanismLigament2d("arm", 1, 90));

  //Physics simulation of the arm
  SingleJointedArmSim physicsSim =
      new SingleJointedArmSim(DCMotor.getKrakenX60(1), ClimbConstants.GEARING, 0, 0, 0, 0, false, 0);
  //Sim state of the TalonFX. 
  TalonFXSimState simMotor = climbMotor.getSimState();
  private final PositionVoltage climbPositionRequest = new PositionVoltage(0.0);
  private final VelocityVoltage climbVelocityRequest = new VelocityVoltage(0.0);

  public ClimbIOSim() {
    //configures base motor
    climbMotor.getConfigurator().apply(getConfiguration());
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    //Sets input voltage from battery
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    //Sets voltage from sim motor
    physicsSim.setInputVoltage(simMotor.getMotorVoltage());

    //Sends data to smartdashboard
    inputs.climbConnected = true;
    inputs.climbAppliedVolts = simMotor.getMotorVoltage();
    inputs.climbCurrentAmps = physicsSim.getCurrentDrawAmps();
    inputs.climbPosition = physicsSim.getAngleRads();
    inputs.climbVelocityRadPerSec = physicsSim.getVelocityRadPerSec();

    physicsSim.update(0.02);

    simMotor.setRawRotorPosition(Radians.of(inputs.climbPosition/ClimbConstants.GEARING));
    simMotor.setRotorVelocity(RadiansPerSecond.of(inputs.climbVelocityRadPerSec/ClimbConstants.GEARING));
    m_climbArm.setAngle(new Rotation2d(inputs.climbPosition));
  }

  @Override
  public void setClimbVelocity(double velocity) {
    climbMotor.setControl(climbVelocityRequest.withVelocity(velocity));
  }

  @Override
  public void setClimbPosition(double position) {
    climbMotor.setControl(climbPositionRequest.withPosition(position));
  }
}

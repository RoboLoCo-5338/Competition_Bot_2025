package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.SimMechanism;

public class ClimbIOSim extends SimMechanism implements ClimbIO {
  @AutoLogOutput(key = "Climb/Mechanism")

  // The logged shape and stuff of the mechanism
  LoggedMechanism2d mechanism = new LoggedMechanism2d(5, 5);

  LoggedMechanismRoot2d root = mechanism.getRoot("climb", 2.5, 0);
  LoggedMechanismLigament2d m_climbArm;

  // Physics simulation of the arm
  SingleJointedArmSim physicsSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          ClimbConstants.GEARING,
          SingleJointedArmSim.estimateMOI(
              ClimbConstants.ARM_LENGTH, 4), // TODO changed for testing, fix massKG
          ClimbConstants.ARM_LENGTH,
          ClimbConstants.MIN_ANGLE,
          ClimbConstants.MAX_ANGLE,
          false,
          ClimbConstants.STARTING_ANGLE);
  // Sim state of the TalonFX.
  TalonFXSimState simMotor = climbMotor.getSimState();

  public ClimbIOSim() {
    super();
    // configures base motor
    climbMotor.getConfigurator().apply(getConfiguration());
    m_climbArm =
        root.append(new LoggedMechanismLigament2d("base", ClimbConstants.BASE_HEIGHT, 90))
            .append(new LoggedMechanismLigament2d("rotator", 0, -90))
            .append(new LoggedMechanismLigament2d("arm", ClimbConstants.ARM_LENGTH, 90));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // Sets input voltage from battery
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    // Sets voltage from sim motor
    physicsSim.setInputVoltage(simMotor.getMotorVoltage());

    physicsSim.update(0.02);
    // Sends data to advantagescope
    inputs.climbConnected = true;
    inputs.climbAppliedVolts = simMotor.getMotorVoltage();
    inputs.climbCurrentAmps = physicsSim.getCurrentDrawAmps();
    inputs.climbPosition = physicsSim.getAngleRads();
    inputs.climbVelocityRadPerSec = physicsSim.getVelocityRadPerSec();
    
    m_climbArm.setAngle(new Rotation2d(inputs.climbPosition));

    simMotor.setRawRotorPosition(Radians.of(physicsSim.getAngleRads() / ClimbConstants.GEARING));
    simMotor.setRotorVelocity(
        RadiansPerSecond.of(physicsSim.getVelocityRadPerSec() / ClimbConstants.GEARING));
  }

  @Override
  public void setClimbVelocity(double velocity) {
    climbMotor.setControl(climbVelocityRequest.withVelocity(velocity));
  }

  @Override
  public void setClimbPosition(double position) {
    climbMotor.setControl(climbPositionRequest.withPosition(position));
  }

  @Override
  public double[] getCurrents() {
    return new double[] {physicsSim.getCurrentDrawAmps()};
  }
}

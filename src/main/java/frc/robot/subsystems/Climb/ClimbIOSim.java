package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class ClimbIOSim implements ClimbIO{
  LoggedMechanism2d mechanism = new LoggedMechanism2d(3, 5);
  LoggedMechanismRoot2d root = mechanism.getRoot("climb", 0, 0);
  LoggedMechanismLigament2d m_climbArm = root.append(new LoggedMechanismLigament2d("arm", 3, 0));
  SingleJointedArmSim physicSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 0, 0, 0, 0, 0, false, 0);
  public ClimbIOSim() {
    Logger.recordOutput("Test", mechanism);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.climbConnected=true;
    inputs.climbAppliedVolts = 0;
    inputs.climbCurrentAmps = 0;
    inputs.climbPosition = Units.rotationsToRadians(climbPosition.getValueAsDouble());
    inputs.climbVelocityRadPerSec = Units.rotationsToRadians(climbPosition.getValueAsDouble());
  }

  @Override
  public void setClimbVelocity(double velocity) {}

  @Override
  public void setClimbPosition(double position) {}

  
}

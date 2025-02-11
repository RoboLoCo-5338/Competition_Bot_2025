package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class ClimbIOSim implements ClimbIO{
  Mechanism2d mechanism = new Mechanism2d(3, 5);
  MechanismRoot2d root = mechanism.getRoot("climb", 0, 0);
  MechanismLigament2d m_climbArm = root.append(new MechanismLigament2d("arm", 0, 0))
  SingleJointedArmSim physicSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 0, 0, 0, 0, 0, false, 0);
  public ClimbIOSim() {
    
  }

  public void updateInputs(ClimbIOInputs inputs) {}

  public void setClimbVelocity(double velocity) {}

  public void setClimbPosition(double position) {}

  
}

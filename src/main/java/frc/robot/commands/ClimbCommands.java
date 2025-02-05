package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;


public class ClimbCommands {


  /** Creates a new ArmCommands. */
  public ClimbCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }


  public static Command setForward() {
    return new InstantCommand(() -> RobotContainer.m_Climb.setForward(0.2), RobotContainer.m_Climb);
  }


  public static Command setBackward() {
    return new InstantCommand(() -> RobotContainer.m_Climb.setBackward(0.2), RobotContainer.m_Climb);
  }


  public static Command stop() {
    return new InstantCommand(() -> RobotContainer.m_Climb.stop(), RobotContainer.m_Climb);
  }
}

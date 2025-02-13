package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.Climb;

public class ClimbCommands {
  //TODO: change the velocities

  public static Command setForward(Climb climb) {
    return climb.setClimbVelocity(1);
  }

  public static Command setBackward(Climb climb) {
    return climb.setClimbVelocity(1);
  }

  public static Command stop(Climb climb) {
    return climb.setClimbVelocity(0);
  }
}

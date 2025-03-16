package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ClimbCommands {

  public static Command setClimbArm(Climb climb, double position) {
    return climb.setClimbPosition(position);
  }

  public static Command moveClimbArm(Climb climb, double speed) {
    return climb.setClimbVelocity(speed);
  }

  public static double getClimbArmPosition(Climb climb) {
    return climb.climbIO.climbMotor.getPosition().getValueAsDouble();
  }
}

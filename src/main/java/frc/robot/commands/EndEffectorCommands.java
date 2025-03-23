package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.endeffector.*;

public class EndEffectorCommands {
  public static Command moveEndEffector(EndEffector endEffector, double speed) {
    return new InstantCommand(() -> endEffector.setEndEffectorVelocity(speed));
  }
  // value is in rotations
  public static double getEndEffectorPosition(EndEffector endEffector) {
    return endEffector.io.endEffectorMotor.getPosition().getValueAsDouble();
  }
}

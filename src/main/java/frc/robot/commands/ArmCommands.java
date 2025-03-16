package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.endeffector.EndEffector;

public class ArmCommands {

  public static Command moveArm(
      Arm arm, EndEffector endEffector, double armSpeed, double endEffectorSpeed) {
    return new ParallelCommandGroup(
        arm.setArmVelocity(() -> armSpeed), endEffector.setEndEffectorVelocity(endEffectorSpeed));
  }

  public static Command moveArm(Arm arm, double speed) {
    return new InstantCommand(() -> arm.setArmVelocity(() -> speed));
    // new InstantCommand(() -> endEffector.setEndEffectorVelocity(speed)));
  }

  public static Command setArm(Arm arm, double position) {
    return arm.setArmPosition(position);
  }

  public static double getArmPosition(Arm arm) {
    return arm.io.armMotor.getAbsoluteEncoder().getPosition();
  }
}

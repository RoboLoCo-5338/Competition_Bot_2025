package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.groundintake.*;

public class GroundIntakeCommands {

  public static Command moveGroundIntakeWheels(GroundIntake groundIntake, double speed) {
    return new InstantCommand(() -> groundIntake.setGroundIntakeVelocity(speed));
  }

  public static Command setGroundIntakeArm(GroundIntake groundIntake, double position) {
    return new InstantCommand(() -> groundIntake.setGroundArmPosition(position));
  }

  // value is in rotations
  public static double getGroundIntakeArmPosition(GroundIntake groundIntake) {
    return groundIntake.io.armMotor.getAbsoluteEncoder().getPosition();
  }

  public static Command moveGroundIntakeArmRaw(GroundIntake groundIntake, double armSpeed) {
    return new InstantCommand(() -> groundIntake.setGroundArmVelocity(armSpeed));
  }

  public static Command moveGroundIntakeArm(
      GroundIntake groundIntake, double armSpeed, double wheelSpeed) {
    return new ParallelCommandGroup(
        moveGroundIntakeArmRaw(groundIntake, armSpeed),
        moveGroundIntakeWheels(groundIntake, wheelSpeed));
  }
}

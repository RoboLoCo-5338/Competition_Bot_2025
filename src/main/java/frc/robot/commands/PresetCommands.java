package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PresetConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;

public class PresetCommands {

  public static Command endEffectorSet(EndEffector endEffector, Arm arm) {
    return new FunctionalCommand(
        () -> System.out.println("end effector set"),
        () -> arm.setArmPosition(0.3),
        (interrupted) -> arm.setArmVelocity(0),
        () -> arm.inputs.armPosition > 0.3,
        arm);
  }

  public static Command presetL2(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(elevator.setElevatorPosition(PresetConstants.elevatorl2));
  }

  public static Command presetL3(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(elevator.setElevatorPosition(PresetConstants.elevatorl3));
  }

  public static Command presetL4(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(elevator.setElevatorPosition(PresetConstants.elevatorl4));
  }

  public static Command stopAll(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        elevator.setElevatorVelocity(() -> 0.0),
        endEffector.setEndEffectorVelocity(0),
        arm.setArmVelocity(0));
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PresetConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;

public class PresetCommands {

  public static Command endEffectorSet(Arm arm) {
    SmartDashboard.putNumber("arm position", arm.getArmPosition().getAsDouble());
    if (arm.getArmPosition().getAsDouble() > 0.310) {
      SmartDashboard.putString("preset2", "we are inside don't do anything case");

      return arm.setArmVelocity(() -> 0);
    } else {
      SmartDashboard.putString("preset2", "we are inside do anything case");
      return arm.setArmPosition(0.310);
    }
  }

  public static Command stowElevator(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        new WaitCommand(0.3).deadlineFor(arm.setArmPosition(0.310)),
        elevator.setElevatorPosition(0),
        arm.setArmPosition(0.310));
  }

  public static Command presetL2(Elevator elevator, EndEffector endEffector, Arm arm) {

    SmartDashboard.putString("preset2", "inside preset functoin");
    return new SequentialCommandGroup(
        new WaitCommand(0.3).deadlineFor(endEffectorSet(arm)),
        elevator.setElevatorPosition(PresetConstants.elevatorl2));
  }

  public static Command presetL3(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
      new WaitCommand(0.3).deadlineFor(endEffectorSet(arm)),
        elevator.setElevatorPosition(PresetConstants.elevatorl3));
  }

  public static Command presetL4(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
      new WaitCommand(0.3).deadlineFor(endEffectorSet(arm)),
        new ParallelCommandGroup(
            arm.setArmPosition(PresetConstants.arml4),
            elevator.setElevatorPosition(PresetConstants.elevatorl4)));
  }

  public static Command net(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        new WaitCommand(0.3).deadlineFor(arm.setArmPosition(0.1)),
        elevator.setElevatorPosition(PresetConstants.elevatorNet));
  }
}

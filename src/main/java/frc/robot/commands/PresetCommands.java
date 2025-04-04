package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.PresetConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endeffector.EndEffector;

public class PresetCommands {

  public static Command endEffectorSet(EndEffector endEffector, Arm arm, double position) {
    return arm.setArmPosition(position)
        .onlyIf(() -> !(arm.getArmPosition().getAsDouble() > position));
  }

  public static Command stowElevator(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        arm.setArmPosition(0.610),
        new WaitCommand(0.3),
        elevator.setElevatorPosition(0.05 * ElevatorConstants.GEARING, 2),
        arm.setArmPosition(0.580));
  }

  public static Command fullIn(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        arm.setArmPosition(0.61),
        new WaitCommand(0.3),
        elevator.setElevatorPosition(0.05 * ElevatorConstants.GEARING, 2));
  }

  public static Command presetL2(Elevator elevator, EndEffector endEffector, Arm arm) {

    SmartDashboard.putString("preset2", "inside preset functoin");
    return new SequentialCommandGroup(
        endEffectorSet(endEffector, arm, 0.61),
        new WaitCommand(0.3),
        elevator.setElevatorPosition(PresetConstants.elevatorl2, 0));
  }

  public static Command presetL3(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        endEffectorSet(endEffector, arm, 0.61),
        new WaitCommand(0.3),
        elevator.setElevatorPosition(PresetConstants.elevatorl3, 0));
  }

  public static Command presetL4(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        endEffectorSet(endEffector, arm, 0.61),
        new ParallelCommandGroup(
            arm.setArmPosition(PresetConstants.arml4),
            elevator.setElevatorPosition(PresetConstants.elevatorl4, 0)));
  }

  public static Command stopAll(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        elevator.setElevatorVelocity(() -> 0.0),
        endEffector.setEndEffectorVelocity(0),
        arm.setArmVelocity(() -> 0));
  }

  public static Command netShoot(Arm arm, EndEffector endEffector) {
    return new ParallelCommandGroup(
        arm.setArmPosition(Constants.PresetConstants.armNet),
        new SequentialCommandGroup(new WaitCommand(0.8), endEffector.setEndEffectorSpeed(-1)));
  }

  public static Command moveEndEffectorLaserCan(EndEffector endEffector) {
    if (endEffector.getIO().getLaserCanMeasurement1() == -1
        || endEffector.getIO().getLaserCanMeasurement2() == -1) {
      return new InstantCommand();
    }
    return new SequentialCommandGroup(
        new RepeatCommand(endEffector.setEndEffectorVelocity(60))
            .until(
                () ->
                    (endEffector.getIO().getLaserCanMeasurement1() < 100
                        && endEffector.getIO().getLaserCanMeasurement2() < 100)),
        // new RepeatCommand(endEffector.setEndEffectorVelocity(60))
        //     .until(
        //         () ->
        //             (endEffector.getIO().getLaserCanMeasurement2() < 100
        //                 && endEffector.getIO().getLaserCanMeasurement1() > 90)),
        // new RepeatCommand(endEffector.setEndEffectorVelocity(60))
        //     .until(
        //         () ->
        //             (endEffector.getIO().getLaserCanMeasurement1() < 100
        //                 && endEffector.getIO().getLaserCanMeasurement2() < 100)),
        endEffector.setEndEffectorVelocity(0.0));
  }
}

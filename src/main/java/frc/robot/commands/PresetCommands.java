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
import frc.robot.subsystems.endeffector.EndEffector;

public class PresetCommands {

  public static Command endEffectorSet(EndEffector endEffector, Arm arm) {
    SmartDashboard.putNumber("arm position", arm.getArmPosition().getAsDouble());
    if (arm.getArmPosition().getAsDouble() > 0.61) {
      SmartDashboard.putString("preset2", "we are inside don't do anything case");

      return arm.setArmVelocity(() -> 0);
    } else {
      SmartDashboard.putString("preset2", "we are inside do anything case");
      return arm.setArmPosition(0.610);
    }
  }

  public static Command stowElevator(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        arm.setArmPosition(0.610),
        new WaitCommand(0.3),
        elevator.setElevatorPosition(0.05),
        arm.setArmPosition(0.580));
  }

  public static Command fullIn(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        arm.setArmPosition(0.61),
        new WaitCommand(0.3),
        elevator.setElevatorPosition(0.05));
  }

  public static Command presetL2(Elevator elevator, EndEffector endEffector, Arm arm) {

    SmartDashboard.putString("preset2", "inside preset functoin");
    return new SequentialCommandGroup(
        endEffectorSet(endEffector, arm),
        new WaitCommand(0.3),
        elevator.setElevatorPosition(PresetConstants.elevatorl2));
  }

  public static Command presetL3(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        endEffectorSet(endEffector, arm),
        new WaitCommand(0.3),
        elevator.setElevatorPosition(PresetConstants.elevatorl3));
  }

  public static Command presetL4(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        endEffectorSet(endEffector, arm),
        new ParallelCommandGroup(
            arm.setArmPosition(PresetConstants.arml4),
            elevator.setElevatorPosition(PresetConstants.elevatorl4)));
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
    System.out.println("Moving end effector");
    if (endEffector.getIO().getLaserCanMeasurement1() == -1
        || endEffector.getIO().getLaserCanMeasurement2() == -1) {
      System.out.println("At least one LaserCAN measurement is broken");
      return new InstantCommand();
    }
    return new SequentialCommandGroup(
        new InstantCommand(() -> System.out.println("0")),
        new RepeatCommand(endEffector.setEndEffectorVelocity(60))
            .until(
                () ->
                    (endEffector.getIO().getLaserCanMeasurement1() < 100
                        && endEffector.getIO().getLaserCanMeasurement2() < 100)),
        new InstantCommand(() -> System.out.println("1")),
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
        endEffector.setEndEffectorVelocity(0.0),
        new InstantCommand(() -> System.out.println("2")));
  }
}

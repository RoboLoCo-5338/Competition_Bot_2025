package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants.ArmPresetConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPresetConstants;
import frc.robot.subsystems.endeffector.EndEffector;

public class PresetCommands {

  public static Command endEffectorSet(EndEffector endEffector, Arm arm, double position) {
    return arm.setArmPosition(position)
        .onlyIf(() -> !(arm.getArmPosition().getAsDouble() > position));
  }

  public static Command stowElevator(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        arm.setArmPosition(-0.264 + 0.09),
        new WaitCommand(0.1),
        elevator.setElevatorPosition(0.05, 2),
        arm.setArmPosition(-0.264));
  }

  public static Command presetL2(Elevator elevator, EndEffector endEffector, Arm arm) {
    SmartDashboard.putString("preset2", "inside preset functoin");
    return new SequentialCommandGroup(
        arm.setArmPosition(ArmPresetConstants.ARM_L2_L3),
        elevator.setElevatorPosition(ElevatorPresetConstants.ELEVATOR_L2, 0));
  }

  public static Command presetL3(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        arm.setArmPosition(ArmPresetConstants.ARM_L2_L3),
        elevator.setElevatorPosition(ElevatorPresetConstants.ELEVATOR_L3, 0));
  }

  public static Command presetL4(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            arm.setArmPosition(ArmPresetConstants.ARM_L4),
            elevator.setElevatorPosition(ElevatorPresetConstants.ELEVATOR_L4, 0)));
  }

  public static Command stopAll(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new ParallelCommandGroup(
        elevator.setElevatorVelocity(() -> 0.0),
        endEffector.setEndEffectorVelocity(0),
        arm.setArmVelocity(() -> 0));
  }

  public static Command netShoot(Arm arm, EndEffector endEffector) {
    return new ParallelCommandGroup(
        arm.setArmPosition(ArmPresetConstants.ARM_NET),
        new SequentialCommandGroup(new WaitCommand(0.8), endEffector.setEndEffectorSpeed(-1)));
  }

  public static Command intakeLaserCan(EndEffector endEffector) {
    return new SequentialCommandGroup(
            new RepeatCommand(endEffector.setEndEffectorVelocity(100))
                .until(
                    () ->
                        (endEffector.getIO().getLaserCanMeasurement1() < 100
                            && endEffector.getIO().getLaserCanMeasurement2() < 100)),
            endEffector.setEndEffectorVelocity(0.0))
        .onlyIf(
            () ->
                !(endEffector.getIO().getLaserCanMeasurement1() == -1
                    || endEffector.getIO().getLaserCanMeasurement2() == -1));
  }

  public static Command outtakeLaserCan(EndEffector endEffector) {
    return new SequentialCommandGroup(
            new RepeatCommand(endEffector.setEndEffectorVelocity(-100))
                .until(
                    () ->
                        (endEffector.getIO().getLaserCanMeasurement1() > 100
                            && endEffector.getIO().getLaserCanMeasurement2() > 100)),
            endEffector.setEndEffectorVelocity(0.0))
        .onlyIf(
            () ->
                endEffector.getIO().getLaserCanMeasurement1() == -1
                    || endEffector.getIO().getLaserCanMeasurement2() == -1);
  }
}

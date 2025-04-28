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

  public static Command endEffectorSet(EndEffector endEffector, Arm arm, double position) {
    //moves arm to a certain position only if it is currently lower than that position
    return arm.setArmPosition(position)
        .onlyIf(() -> !(arm.getArmPosition().getAsDouble() > position));
  }

  public static Command stowElevator(Elevator elevator, EndEffector endEffector, Arm arm) {
    //moves arm to 0.51, then uses the slot 2 pid/feedforward to move the elevator to 0.05, then moves arm more down
    return new SequentialCommandGroup(
        arm.setArmPosition(0.51),
        new WaitCommand(0.1),
        elevator.setElevatorPosition(0.05, 2),
        arm.setArmPosition(0.425));
  }

  public static Command presetL2(Elevator elevator, EndEffector endEffector, Arm arm) {
    //L2 preset
    SmartDashboard.putString("preset2", "inside preset functoin");
    return new SequentialCommandGroup(
        arm.setArmPosition(0.54), elevator.setElevatorPosition(PresetConstants.elevatorl2, 0));
  }

  public static Command presetL3(Elevator elevator, EndEffector endEffector, Arm arm) {
    //L3 preset
    return new SequentialCommandGroup(
        arm.setArmPosition(0.54), elevator.setElevatorPosition(PresetConstants.elevatorl3, 0));
  }

  public static Command presetL4(Elevator elevator, EndEffector endEffector, Arm arm) {
    //sequential group not needed I think
    return new SequentialCommandGroup(
        //simultaneously sets arm position and elevator position
        new ParallelCommandGroup(
            arm.setArmPosition(PresetConstants.arml4),
            elevator.setElevatorPosition(PresetConstants.elevatorl4, 0)));
  }

  public static Command stopAll(Elevator elevator, EndEffector endEffector, Arm arm) {
    //stops all
    return new SequentialCommandGroup(
        elevator.setElevatorVelocity(() -> 0.0),
        endEffector.setEndEffectorVelocity(0),
        arm.setArmVelocity(() -> 0));
  }

  public static Command netShoot(Arm arm, EndEffector endEffector) {
    return new ParallelCommandGroup(
      //simulatnelously sets arm position and then after a little shoots out the end effector (probably under development)
        arm.setArmPosition(Constants.PresetConstants.armNet),
        new SequentialCommandGroup(new WaitCommand(0.8), endEffector.setEndEffectorSpeed(-1)));
  }
  //intake using laser can (note that this does not work for L2, L3 because it goes until it detects coral (automatically detects coral if trying to outtake))
  public static Command moveEndEffectorLaserCan(EndEffector endEffector) {
    //if either laser can is not working do nothing
    if (endEffector.getIO().getLaserCanMeasurement1() == -1
        || endEffector.getIO().getLaserCanMeasurement2() == -1) {
      return new InstantCommand();
    }
    return new SequentialCommandGroup(
        new RepeatCommand(endEffector.setEndEffectorVelocity(100))
        //intakes until both laser cans detect coral
            .until(
                () ->
                    (endEffector.getIO().getLaserCanMeasurement1() < 100
                        && endEffector.getIO().getLaserCanMeasurement2() < 100)),
        endEffector.setEndEffectorVelocity(0.0));
  }
  //same thing as above but for L4 it seems (terrible naming for these two) (note that this does not work for L2, L3 b/c end effector velocity is opposite)
  public static Command outtakeLaserCan(EndEffector endEffector) {
    if (endEffector.getIO().getLaserCanMeasurement1() == -1
        || endEffector.getIO().getLaserCanMeasurement2() == -1) {
      return new InstantCommand();
    }
    return new SequentialCommandGroup(
        new RepeatCommand(endEffector.setEndEffectorVelocity(-100))
            .until(
                () ->
                    (endEffector.getIO().getLaserCanMeasurement1() > 100
                        && endEffector.getIO().getLaserCanMeasurement2() > 100)),
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

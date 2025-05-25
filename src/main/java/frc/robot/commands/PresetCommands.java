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

    /**
     * Sets arm to a certain position only if it is currently lower than that position
     * @param endEffector
     * @param arm
     * @param position
     * @return StartEndCommand (ends when arm is within tolerance)
     */
  public static Command endEffectorSet(EndEffector endEffector, Arm arm, double position) {
    //moves arm to a certain position only if it is currently lower than that position
    return arm.setArmPosition(position)
        .onlyIf(() -> !(arm.getArmPosition().getAsDouble() > position));
  }

  /**
   * Drops arm and elevator down
   * @param elevator
   * @param endEffector
   * @param arm
   * @return SequentialCommandGroup
   */
  public static Command stowElevator(Elevator elevator, EndEffector endEffector, Arm arm) {
    //moves arm to 0.51, then uses the slot 2 pid/feedforward to move the elevator to 0.05, then moves arm more down
    return new SequentialCommandGroup(
        arm.setArmPosition(0.51),
        new WaitCommand(0.1),
        elevator.setElevatorPosition(0.05, 2),
        arm.setArmPosition(0.42));
  }

  /**
   * Sets arm position then elevator position to L2
   * @param elevator
   * @param endEffector
   * @param arm
   * @return SequentialCommandGroup
   */
  public static Command presetL2(Elevator elevator, EndEffector endEffector, Arm arm) {
    //L2 preset
    SmartDashboard.putString("preset2", "inside preset functoin");
    return new SequentialCommandGroup(
        arm.setArmPosition(ArmPresetConstants.ARM_L2_L3),
        elevator.setElevatorPosition(ElevatorPresetConstants.ELEVATOR_L2, 0));
  }

  /**
   * Sets arm position then elevator position to L3
   * @param elevator
   * @param endEffector
   * @param arm
   * @return SequentialCommandGroup
   */
  public static Command presetL3(Elevator elevator, EndEffector endEffector, Arm arm) {
    //L3 preset
    return new SequentialCommandGroup(
        arm.setArmPosition(ArmPresetConstants.ARM_L2_L3),
        elevator.setElevatorPosition(ElevatorPresetConstants.ELEVATOR_L3, 0));
  }

  /**
   * Simultaneously sets arm position and elevator position to L4
   * @param elevator
   * @param endEffector
   * @param arm
   * @return ParallelCommandGroup wrapped in a SequentialCommandGroup
   */
  public static Command presetL4(Elevator elevator, EndEffector endEffector, Arm arm) {
    //sequential group not needed I think
    return new SequentialCommandGroup(
        //simultaneously sets arm position and elevator position
        new ParallelCommandGroup(
            arm.setArmPosition(ArmPresetConstants.ARM_L4),
            elevator.setElevatorPosition(ElevatorPresetConstants.ELEVATOR_L4, 0)));
  }

  /**
   * Sets elevator, endEffector, and arm velocities to 0
   * @param elevator
   * @param endEffector
   * @param arm
   * @return SequentialCommandGroup
   */
  public static Command stopAll(Elevator elevator, EndEffector endEffector, Arm arm) {
    //stops all
    return new SequentialCommandGroup(
        elevator.setElevatorVelocity(() -> 0.0),
        endEffector.setEndEffectorVelocity(0),
        arm.setArmVelocity(() -> 0));
  }

  /**
   * Moves the arm up while endEffector shoots (mech tech net shoot)
   * @param arm
   * @param endEffector
   * @return ParallelCommandGroup
   */
  public static Command netShoot(Arm arm, EndEffector endEffector) {
    return new ParallelCommandGroup(
        arm.setArmPosition(ArmPresetConstants.ARM_NET),
        new SequentialCommandGroup(new WaitCommand(0.8), endEffector.setEndEffectorSpeed(-1)));
  }

  /**
   * Intakes until lasercans detect coral, then stops intake
   * @param endEffector
   * @return SequentialCommandGroup
   */
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
            //only stops end effector if at least one lasercan is present (I don't see the need though)
                !(endEffector.getIO().getLaserCanMeasurement1() == -1
                    || endEffector.getIO().getLaserCanMeasurement2() == -1));
  }
  //same thing as above but for L4 it seems (terrible naming for these two) (note that this does not work for L2, L3 b/c end effector velocity is opposite)
  /**
   * L4 outake until lasercans no longer detect coral
   * @param endEffector
   * @return SequentialCommandGroup
   */
  public static Command outtakeLaserCan(EndEffector endEffector) {
    return new SequentialCommandGroup(
            new RepeatCommand(endEffector.setEndEffectorVelocity(-100))
                .until(
                    () ->
                        (endEffector.getIO().getLaserCanMeasurement1() > 100
                            && endEffector.getIO().getLaserCanMeasurement2() > 100)),
            endEffector.setEndEffectorVelocity(0.0))
        .onlyIf(
             //only stops end effector if at least one lasercan is present (I don't see the need though)

            () ->
                endEffector.getIO().getLaserCanMeasurement1() == -1
                    || endEffector.getIO().getLaserCanMeasurement2() == -1);
  }
}

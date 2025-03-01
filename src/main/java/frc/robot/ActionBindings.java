package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.EndEffectorCommands;
import frc.robot.commands.GroundIntakeCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.led.LED;

public class ActionBindings {
  public static Drive drive;
  public static CommandXboxController driveController;
  public static CommandXboxController operatorController;
  public static double exponentialVariable = 25.0;
  public static Climb climb;
  public static Elevator elevator;
  public static GroundIntake groundIntake;
  public static EndEffector endEffector;
  public static LED led;
  public static Arm arm;

  public static Command lockToZero() {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -driveController.getLeftY(),
        () -> -driveController.getLeftX(),
        () -> new Rotation2d());
  }

  public static Command climbPreset() {
    return ButtonBindings.blankCommand("climbPreset");
  }

  public static Command gyroReset() {
    return Commands.runOnce(
            () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
            drive)
        .ignoringDisable(true);
  }

  public static Command manualClimbDown() {
    return ClimbCommands.moveClimbArm(climb, -0.4);
  }

  public static Command stopClimb() {
    return ClimbCommands.moveClimbArm(climb, 0);
  }

  public static Command manualClimbUp() {
    return ClimbCommands.moveClimbArm(climb, 0.4);
  }

  public static Command manualElevatorStop() {
    return ElevatorCommands.moveElevator(elevator, 0);
  }

  public static Command manualElevatorUp() {
    return ElevatorCommands.moveElevator(elevator, 0.4);
  }

  public static Command manualElevatorDown() {
    return ElevatorCommands.moveElevator(elevator, -0.4);
  }

  public static Command l3Preset() {
    return ButtonBindings.blankCommand("l3Preset");
  }

  public static Command elevatorSlow() {
    return ButtonBindings.blankCommand("elevatorSlow");
  }

  public static Command l2Preset() {
    return ButtonBindings.blankCommand("l2Preset");
  }

  public static Command l4Preset() {
    return ButtonBindings.blankCommand("l4Preset");
  }

  public static Command elevatorFast() {
    return ButtonBindings.blankCommand("elevatorFast");
  }

  public static Command manualArmDown() {
    return ArmCommands.moveArm(arm, -0.4);
  }

  public static Command manualArmUp() {
    return ArmCommands.moveArm(arm, 0.4);
  }

  public static Command manualArmStop() {
    return ArmCommands.moveArm(arm, 0);
  }

  public static Command netPreset() {
    return ButtonBindings.blankCommand("netPreset");
  }

  public static Command groundIntakeIn() {
    return GroundIntakeCommands.moveGroundIntakeArm(groundIntake, -0.4, 0.4);
  }

  public static Command groundIntakeOut() {
    return GroundIntakeCommands.moveGroundIntakeArm(groundIntake, 0.4, -0.4);
  }

  public static Command groundIntakeStop() {
    return GroundIntakeCommands.moveGroundIntakeArm(groundIntake, 0, 0);
  }

  public static Command endEffectorIn() {
    return EndEffectorCommands.moveEndEffector(endEffector, 0.4);
  }

  public static Command endEffectorOut() {
    return EndEffectorCommands.moveEndEffector(endEffector, -0.4);
  }

  public static Command endEffectorStop() {
    return EndEffectorCommands.moveEndEffector(endEffector, 0);
  }

  public static Command groundIntakeSlow() {
    return ButtonBindings.blankCommand("groundIntakeSlow");
  }

  public static Command groundIntakeFast() {
    return ButtonBindings.blankCommand("groundIntakeFast");
  }
}

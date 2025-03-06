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
  public Drive drive;
  public CommandXboxController driveController;
  public CommandXboxController operatorController;
  public double exponentialVariable = 25.0;
  public Climb climb;
  public Elevator elevator;
  public GroundIntake groundIntake;
  public EndEffector endEffector;
  public LED led;
  public Arm arm;

  public ActionBindings(
      Drive drive,
      CommandXboxController driveController,
      CommandXboxController operatorController,
      Climb climb,
      Elevator elevator,
      GroundIntake groundIntake,
      EndEffector endEffector,
      LED led,
      Arm arm) {
    this.drive = drive;
    this.driveController = driveController;
    this.operatorController = operatorController;
    this.climb = climb;
    this.elevator = elevator;
    this.groundIntake = groundIntake;
    this.endEffector = endEffector;
    this.led = led;
    this.arm = arm;
  }

  public Command lockToZero() {
    return DriveCommands.joystickDriveAtAngle(
        this.drive,
        () -> -driveController.getLeftY(),
        () -> -driveController.getLeftX(),
        () -> new Rotation2d());
  }

  public Command climbPreset() {
    return ButtonBindings.blankCommand("climbPreset");
  }

  public Command gyroReset() {
    return Commands.runOnce(
            () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
            drive)
        .ignoringDisable(true);
  }

  public Command manualClimbDown() {
    return ClimbCommands.moveClimbArm(climb, -0.4);
  }

  public Command stopClimb() {
    return ClimbCommands.moveClimbArm(climb, 0);
  }

  public Command manualClimbUp() {
    return ClimbCommands.moveClimbArm(climb, 0.4);
  }

  public Command manualElevatorStop() {
    return ElevatorCommands.moveElevator(elevator, 0);
  }

  public Command manualElevatorUp() {
    return ElevatorCommands.moveElevator(elevator, 0.4);
  }

  public Command manualElevatorDown() {
    return ElevatorCommands.moveElevator(elevator, -0.4);
  }

  public Command l3Preset() {
    return ButtonBindings.blankCommand("l3Preset");
  }

  public Command elevatorSlow() {
    return ButtonBindings.blankCommand("elevatorSlow");
  }

  public Command l2Preset() {
    return ButtonBindings.blankCommand("l2Preset");
  }

  public Command l4Preset() {
    return ButtonBindings.blankCommand("l4Preset");
  }

  public Command elevatorFast() {
    return ButtonBindings.blankCommand("elevatorFast");
  }

  public Command manualArmDown() {
    return ArmCommands.moveArm(arm, -0.4);
  }

  public Command manualArmUp() {
    return ArmCommands.moveArm(arm, 0.4);
  }

  public Command manualArmStop() {
    return ArmCommands.moveArm(arm, 0);
  }

  public Command netPreset() {
    return ButtonBindings.blankCommand("netPreset");
  }

  public Command groundIntakeIn() {
    return GroundIntakeCommands.moveGroundIntakeArm(groundIntake, -0.4, 0.4);
  }

  public Command groundIntakeOut() {
    return GroundIntakeCommands.moveGroundIntakeArm(groundIntake, 0.4, -0.4);
  }

  public Command groundIntakeStop() {
    return GroundIntakeCommands.moveGroundIntakeArm(groundIntake, 0, 0);
  }

  public Command endEffectorIn() {
    return EndEffectorCommands.moveEndEffector(endEffector, 0.4);
  }

  public Command endEffectorOut() {
    return EndEffectorCommands.moveEndEffector(endEffector, -0.4);
  }

  public Command endEffectorStop() {
    return EndEffectorCommands.moveEndEffector(endEffector, 0);
  }

  public Command groundIntakeSlow() {
    return ButtonBindings.blankCommand("groundIntakeSlow");
  }

  public Command groundIntakeFast() {
    return ButtonBindings.blankCommand("groundIntakeFast");
  }
}

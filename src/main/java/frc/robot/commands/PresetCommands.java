package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.PresetConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
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
        elevator.setElevatorPosition(1.0),
        arm.setArmPosition(0.580));
  }

  public static Command printDebug(String message) {
    return new InstantCommand(() -> System.out.println(message));
  }

  public static Command fullIn(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        arm.setArmPosition(0.61), new WaitCommand(0.3), elevator.setElevatorPosition(0.05));
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

  public static Command prefireMechTech(Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(presetL2(elevator, endEffector, arm));
  }

  // 8.79m in x direction is where the barge is
  public static Trigger shootMechTech(
      double offset, Drive drive, CommandXboxController controller) {
    // trigger to figure out when to shoot
    return new Trigger(
        () -> {
          double gravity = 9.81;
          double ball_apex_height = 0.6094; // 2 feet. 0.6094 meters
          double x_distance = Math.abs(drive.getPose().getTranslation().getX() - 8.79);
          double x_speed = Math.abs(drive.getChassisSpeeds().vxMetersPerSecond);
          double y_speed = Math.sqrt(2 * gravity * ball_apex_height);
          double y_needed_height = 2.4; // in meters
          double y_released_height = 2.1463;
          double del_y = y_needed_height - y_released_height;
          double t_net =
              (-y_speed + Math.sqrt(Math.pow(y_speed, 2) + 2 * gravity * del_y)) / -gravity;

          double delx = (x_speed * t_net) + offset;

          if (Math.abs(x_distance - delx) < 0.4 && controller.leftBumper().getAsBoolean()) {

            return true;
          }

          return false;
        });
  }

  public static SequentialCommandGroup initMechTechShot(
      Elevator elevator, EndEffector endEffector, Arm arm) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> System.out.println("Doing mechtech shot!")),
        presetL4(elevator, endEffector, arm),
        endEffector.setEndEffectorVelocity(-100));
  }
  // public static SequentialCommandGroup doMechTechShot(
  //     EndEffector endEffector, Drive drive, Elevator elevator, Arm arm) {}

  public static InstantCommand changeDriveSpeedTo(Drive drive, double speed) {
    return new InstantCommand(() -> DriveCommands.setSpeed(speed));
  }

  public static Command moveEndEffectorLaserCan(EndEffector endEffector) {
    if (endEffector.getIO().getLaserCanMeasurement1() == -1
        || endEffector.getIO().getLaserCanMeasurement2() == -1) {
      return new InstantCommand();
    }
    return new SequentialCommandGroup(
        new RepeatCommand(endEffector.setEndEffectorVelocity(60))
            .onlyWhile(
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

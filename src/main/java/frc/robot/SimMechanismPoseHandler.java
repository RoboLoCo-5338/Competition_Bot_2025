package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmPresetConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPresetConstants;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorConstants;

public class SimMechanismPoseHandler {
    Drive drive;
    Elevator elevator;
    Arm arm;
    EndEffector endEffector;
    private static double last_roller_position = 0.0;
    private static double time_elapsed = 0.0;
    public SimMechanismPoseHandler(Drive drive, Elevator elevator, Arm arm, EndEffector endEffector) {
        this.drive = drive;
        this.elevator = elevator;
        this.arm = arm;
        this.endEffector = endEffector;
    }
    
  
  @AutoLogOutput(key = "Odometry/ElevatorStage1")
  public Pose3d getElevatorStage1SimPose() {
    return new Pose3d(
        ElevatorConstants.ElevatorSimConstants.STAGE_1_ORIGIN_X,
        ElevatorConstants.ElevatorSimConstants.STAGE_1_ORIGIN_Y,
        ElevatorConstants.ElevatorSimConstants.STAGE_1_ORIGIN_Z
            + elevator.getElevatorPosition() * 0.5,
        new Rotation3d());
  }

  @AutoLogOutput(key = "Odometry/ElevatorStage2")
  public Pose3d getElevatorStage2SimPose() {
    return new Pose3d(
        ElevatorConstants.ElevatorSimConstants.STAGE_2_ORIGIN_X,
        ElevatorConstants.ElevatorSimConstants.STAGE_2_ORIGIN_Y,
        ElevatorConstants.ElevatorSimConstants.STAGE_2_ORIGIN_Z
            + elevator.getElevatorPosition() * 1,
        new Rotation3d());
  }

  @AutoLogOutput(key = "Odometry/EndEffector")
  public Pose3d getEndEffectorSimPose() {
    return new Pose3d(
        ArmConstants.ArmSimConstants.ORIGIN_X,
        ArmConstants.ArmSimConstants.ORIGIN_Y,
        ArmConstants.ArmSimConstants.ORIGIN_Z + elevator.getElevatorPosition() * 1,
        new Rotation3d(
            0.0,
            -1
                * (Units.rotationsToRadians(arm.getArmPosition())
                    + ArmConstants.ArmSimConstants.STARTING_ANGLE
                    + Units.degreesToRadians(90)),
            0.0));
  }

  public static Pose3d getEndEffectorCoralSimPose(Elevator elevator, Arm arm) {
    double elevatorHeight = elevator.getElevatorPosition() * 1;
    double endEffectorRotation =
        -1
            * (Units.rotationsToRadians(arm.getArmPosition())
                + ArmConstants.ArmSimConstants.STARTING_ANGLE
                + Units.degreesToRadians(90));

    double offset_x =
        EndEffectorConstants.EndEffectorSimConstants.FRONT_ROLLER_ORIGIN_X
            - ArmConstants.ArmSimConstants.ORIGIN_X
            - 0.05;
    double offset_z =
        EndEffectorConstants.EndEffectorSimConstants.FRONT_ROLLER_ORIGIN_Z
            - ArmConstants.ArmSimConstants.ORIGIN_Z
            - 0.1;

    double rotated_x =
        offset_x * Math.cos(endEffectorRotation) + offset_z * Math.sin(endEffectorRotation);
    double rotated_z =
        -offset_x * Math.sin(endEffectorRotation) + offset_z * Math.cos(endEffectorRotation);

    double final_x = rotated_x + ArmConstants.ArmSimConstants.ORIGIN_X;
    double final_z = rotated_z + ArmConstants.ArmSimConstants.ORIGIN_Z + elevatorHeight;
    return new Pose3d(
        final_x,
        EndEffectorConstants.EndEffectorSimConstants.FRONT_ROLLER_ORIGIN_Y,
        final_z,
        new Rotation3d(
            0.0,
            Math.PI / 2
                - Units.rotationsToRadians(arm.getArmPosition())
                + Units.degreesToRadians(27),
            0.0));
  }

  @AutoLogOutput(key = "Odometry/EndEffectorFrontRoller")
  public Pose3d getEndEffectorFrontRollerSimPose() {
    double elevatorHeight = elevator.getElevatorPosition() * 1;
    double endEffectorRotation =
        -1
            * (Units.rotationsToRadians(arm.getArmPosition())
                + ArmConstants.ArmSimConstants.STARTING_ANGLE
                + Units.degreesToRadians(90));

    double offset_x =
        EndEffectorConstants.EndEffectorSimConstants.FRONT_ROLLER_ORIGIN_X
            - ArmConstants.ArmSimConstants.ORIGIN_X;
    double offset_z =
        EndEffectorConstants.EndEffectorSimConstants.FRONT_ROLLER_ORIGIN_Z
            - ArmConstants.ArmSimConstants.ORIGIN_Z;

    double rotated_x =
        offset_x * Math.cos(endEffectorRotation) + offset_z * Math.sin(endEffectorRotation);
    double rotated_z =
        -offset_x * Math.sin(endEffectorRotation) + offset_z * Math.cos(endEffectorRotation);

    double final_x = rotated_x + ArmConstants.ArmSimConstants.ORIGIN_X;
    double final_z = rotated_z + ArmConstants.ArmSimConstants.ORIGIN_Z + elevatorHeight;

    // System.out.println("Speed: " + endEffector.getEndEffectorVelocity());
    last_roller_position +=
        (endEffector.getEndEffectorVelocity() / 10.0 * (Timer.getTimestamp() - time_elapsed));
    time_elapsed = Timer.getTimestamp();
    // System.out.println("Time elapsed: " + Timer.getTimestamp());
    last_roller_position %= 2.0 * Math.PI;
    // System.out.println("Pos2: " + last_roller_position);
    return new Pose3d(
        final_x,
        EndEffectorConstants.EndEffectorSimConstants.FRONT_ROLLER_ORIGIN_Y,
        final_z,
        new Rotation3d(0.0, last_roller_position, 0.0));
  }

  @AutoLogOutput(key = "Odometry/EndEffectorBackRoller")
  public Pose3d getEndEffectorBackRollerSimPose() {
    double elevatorHeight = elevator.getElevatorPosition() * 1;
    double endEffectorRotation =
        -1
            * (Units.rotationsToRadians(arm.getArmPosition())
                + ArmConstants.ArmSimConstants.STARTING_ANGLE
                + Units.degreesToRadians(90));

    double offset_x =
        EndEffectorConstants.EndEffectorSimConstants.BACK_ROLLER_ORIGIN_X
            - ArmConstants.ArmSimConstants.ORIGIN_X;
    double offset_z =
        EndEffectorConstants.EndEffectorSimConstants.BACK_ROLLER_ORIGIN_Z
            - ArmConstants.ArmSimConstants.ORIGIN_Z;

    double rotated_x =
        offset_x * Math.cos(endEffectorRotation) + offset_z * Math.sin(endEffectorRotation);
    double rotated_z =
        -offset_x * Math.sin(endEffectorRotation) + offset_z * Math.cos(endEffectorRotation);

    double final_x = rotated_x + ArmConstants.ArmSimConstants.ORIGIN_X;
    double final_z = rotated_z + ArmConstants.ArmSimConstants.ORIGIN_Z + elevatorHeight;

    return new Pose3d(
        final_x,
        EndEffectorConstants.EndEffectorSimConstants.BACK_ROLLER_ORIGIN_Y,
        final_z,
        new Rotation3d(0.0, last_roller_position, 0.0));
  }

  @AutoLogOutput(key = "Odometry/BackLeftSwerveModule")
  public Pose3d getBackLeftSwerveModuleSimPose() {
    return new Pose3d(
        DriveConstants.DriveSimConstants.BACK_LEFT_SWERVE_MODULE_ORIGIN_X,
        DriveConstants.DriveSimConstants.BACK_LEFT_SWERVE_MODULE_ORIGIN_Y,
        DriveConstants.DriveSimConstants.BACK_LEFT_SWERVE_MODULE_ORIGIN_Z,
        new Rotation3d(0.0, 0.0, drive.getModulePositions()[2].angle.getRadians()));
  }

  @AutoLogOutput(key = "Odometry/BackRightSwerveModule")
  public Pose3d getBackRightSwerveModuleSimPose() {
    return new Pose3d(
        DriveConstants.DriveSimConstants.BACK_RIGHT_SWERVE_MODULE_ORIGIN_X,
        DriveConstants.DriveSimConstants.BACK_RIGHT_SWERVE_MODULE_ORIGIN_Y,
        DriveConstants.DriveSimConstants.BACK_RIGHT_SWERVE_MODULE_ORIGIN_Z,
        new Rotation3d(0.0, 0.0, drive.getModulePositions()[3].angle.getRadians()));
  }

  @AutoLogOutput(key = "Odometry/FrontLeftSwerveModule")
  public Pose3d getFrontLeftSwerveModuleSimPose() {
    return new Pose3d(
        DriveConstants.DriveSimConstants.FRONT_LEFT_SWERVE_MODULE_ORIGIN_X,
        DriveConstants.DriveSimConstants.FRONT_LEFT_SWERVE_MODULE_ORIGIN_Y,
        DriveConstants.DriveSimConstants.FRONT_LEFT_SWERVE_MODULE_ORIGIN_Z,
        new Rotation3d(0.0, 0.0, drive.getModulePositions()[0].angle.getRadians()));
  }

  @AutoLogOutput(key = "Odometry/FrontRightSwerveModule")
  public Pose3d getFrontRightSwerveModuleSimPose() {
    return new Pose3d(
        DriveConstants.DriveSimConstants.FRONT_RIGHT_SWERVE_MODULE_ORIGIN_X,
        DriveConstants.DriveSimConstants.FRONT_RIGHT_SWERVE_MODULE_ORIGIN_Y,
        DriveConstants.DriveSimConstants.FRONT_RIGHT_SWERVE_MODULE_ORIGIN_Z,
        new Rotation3d(0.0, 0.0, drive.getModulePositions()[1].angle.getRadians()));
  }

  @AutoLogOutput(key = "Odometry/BackLeftWheel")
  public Pose3d getBackLeftWheelSimPose() {
    return new Pose3d(
        DriveConstants.DriveSimConstants.BACK_LEFT_WHEEL_ORIGIN_X,
        DriveConstants.DriveSimConstants.BACK_LEFT_WHEEL_ORIGIN_Y,
        DriveConstants.DriveSimConstants.BACK_LEFT_WHEEL_ORIGIN_Z,
        new Rotation3d(
            0.0,
            drive.getWheelRadiusCharacterizationPositions()[2],
            drive.getModulePositions()[2].angle.getRadians()));
  }

  @AutoLogOutput(key = "Odometry/BackRightWheel")
  public Pose3d getBackRightWheelSimPose() {
    return new Pose3d(
        DriveConstants.DriveSimConstants.BACK_RIGHT_WHEEL_ORIGIN_X,
        DriveConstants.DriveSimConstants.BACK_RIGHT_WHEEL_ORIGIN_Y,
        DriveConstants.DriveSimConstants.BACK_RIGHT_WHEEL_ORIGIN_Z,
        new Rotation3d(
            0.0,
            drive.getWheelRadiusCharacterizationPositions()[3],
            drive.getModulePositions()[3].angle.getRadians()));
  }

  @AutoLogOutput(key = "Odometry/FrontLeftWheel")
  public Pose3d getFrontLeftWheelSimPose() {
    return new Pose3d(
        DriveConstants.DriveSimConstants.FRONT_LEFT_WHEEL_ORIGIN_X,
        DriveConstants.DriveSimConstants.FRONT_LEFT_WHEEL_ORIGIN_Y,
        DriveConstants.DriveSimConstants.FRONT_LEFT_WHEEL_ORIGIN_Z,
        new Rotation3d(
            0.0,
            drive.getWheelRadiusCharacterizationPositions()[0],
            drive.getModulePositions()[0].angle.getRadians()));
  }

  @AutoLogOutput(key = "Odometry/FrontRightWheel")
  public Pose3d getFrontRightWheelModuleSimPose() {
    return new Pose3d(
        DriveConstants.DriveSimConstants.FRONT_RIGHT_WHEEL_ORIGIN_X,
        DriveConstants.DriveSimConstants.FRONT_RIGHT_WHEEL_ORIGIN_Y,
        DriveConstants.DriveSimConstants.FRONT_RIGHT_WHEEL_ORIGIN_Z,
        new Rotation3d(
            0.0,
            drive.getWheelRadiusCharacterizationPositions()[1],
            drive.getModulePositions()[1].angle.getRadians()));
  }

    public boolean stowed() {
    return Math.abs(arm.getArmPosition() - ArmPresetConstants.ARM_STOW_FINAL)
            < ArmConstants.POSITION_TOLERANCE
        && Math.abs(elevator.getElevatorPosition() - ElevatorPresetConstants.ELEVATOR_STOW)
            < ElevatorConstants.POSITION_TOLERANCE;
  }
}

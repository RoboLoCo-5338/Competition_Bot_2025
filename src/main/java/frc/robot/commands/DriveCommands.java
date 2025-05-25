// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.vision.VisionConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  //checks if the alliance is blue or red (blue is false, red is true)
  public static boolean isFlipped =
      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

  public static double slowMode = 1;

  private DriveCommands() {}
  /**
   * Calculates the new vector based on joystick input
   * @param x from joystick
   * @param y from joystick
   * @return Translation 2d of the vector created (resulting vector magnitude is min 0, max 1)
   */
  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband of 0.06 as well as get magnitude using hypotenuse (vectors)
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.DEADBAND);
    //uses tangent to find angle of vector
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(
                  xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode);

          // Apply rotation deadband
          double omega =
              MathUtil.applyDeadband(
                  omegaSupplier.getAsDouble() * slowMode, DriveConstants.DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          drive.runVelocity(
            //turns field relative to robot relative
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate next angular speed output based on current angle, and I think since it's a profiledPIDController w/ trapezoid profile,
              // it will start to decelerate once the current angle is close enough to the target angle
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              drive.runVelocity(
                  //turns field relative to robot relative
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(DriveConstants.FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * DriveConstants.FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(DriveConstants.WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(DriveConstants.WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }
  /**
   * Locks rotation onto the center of the reef
   *
   * @param drive Drivetrain
   * @param xSupplier Supplier of x velocity
   * @param ySupplier Supplier of y velocity
   * @return Command to strafe around the reef center
   */
  public static Command reefStrafe(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        //rotationSupplier
        () -> {
          //robot pose
          Translation2d robot = drive.getPose().getTranslation();
          //reef pose
          Translation2d reef =
              (isFlipped) ? new Translation2d(13.06185, 4.03) : new Translation2d(4.5, 4.03);
          //angle to get to reef based on robot position
          return new Rotation2d(Math.atan2(reef.getY() - robot.getY(), reef.getX() - robot.getX()));
        });
  }

  /**
   * Paths to one of the destinations on the field
   *
   * @param drive Drivetrain
   * @param destination Destination that the robot paths to
   * @return Command that makes the robot path to the destination
   */
  public static Command pathToDestination(
      Drive drive,
      Supplier<PathDestination> destination,
      CommandXboxController driverController,
      Direction direction,
      LED led) {
    return new DeferredCommand(
        () -> {
          Pose2d targetPose = destination.get().getTargetPosition();
          Logger.recordOutput("Path to Destination", drive.getPose().log(targetPose));

          return new Command() {

            @Override
            public void initialize() {
              drive.autoXDriveController.reset();
              drive.autoYDriveController.reset();
              drive.autoTurnController.reset();

              drive.autoXDriveController.setTolerance(0.01);
              drive.autoYDriveController.setTolerance(0.01);
              drive.autoTurnController.setTolerance(0.025);
              drive.autoXDriveController.setSetpoint(targetPose.getX());
              drive.autoYDriveController.setSetpoint(targetPose.getY());
              drive.autoTurnController.setSetpoint(targetPose.getRotation().getRadians());
            }

            @Override
            public void execute() {
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      new ChassisSpeeds(
                          drive.autoXDriveController.calculate(drive.getPose().getX()),
                          drive.autoYDriveController.calculate(drive.getPose().getY()),
                          drive.autoTurnController.calculate(
                              drive.getPose().getRotation().getRadians())),
                      drive.getPose().getRotation()));
            }

            @Override
            public boolean isFinished() {
              return (drive.autoXDriveController.atSetpoint()
                  && drive.autoYDriveController.atSetpoint()
                  && drive.autoTurnController.atSetpoint());
            }

            @Override
            public void end(boolean interrupted) {
              drive.runVelocity(new ChassisSpeeds(0, 0, 0));
              led.alignEndFlash(
                      !(drive.autoXDriveController.atSetpoint()
                          && drive.autoYDriveController.atSetpoint()
                          && drive.autoTurnController.atSetpoint()))
                  .schedule();
            }
          }.onlyIf(
              () ->
                  Math.sqrt(
                          targetPose.minus(drive.getPose()).getX()
                                  * targetPose.minus(drive.getPose()).getX()
                              + targetPose.minus(drive.getPose()).getY()
                                  * targetPose.minus(drive.getPose()).getY())
                      < 3);
        },
        new HashSet<Subsystem>() {
          {
            //required subsystem
            add(drive);
          }
        });
  }

  /**
   * Util function to flip the pose of the alliance based on the alliance
   *
   * @param pose Pose to flip
   * @return Flipped Pose
   */
  public static Pose2d allianceFlip(Pose2d pose) {
    return (isFlipped) ? FlippingUtil.flipFieldPose(pose) : pose;
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  /** Abstract class for destinations to path to. */
  public abstract static class PathDestination {
    /**
     * Gets the pose the robot should path to for reaching a certain destination.
     *
     * @return The pose the robot needs to path to.
     */
    public abstract Pose2d getTargetPosition();
  }

  /** Processor destination. */
  public static class Processor extends PathDestination {
    public Processor() {}

    @Override
    public Pose2d getTargetPosition() {
      //returns processor possession depending on the alliance
      return allianceFlip(new Pose2d(5.980, 0.532, new Rotation2d()));
    }
  }

  /**
   * Coral Station destination. If a direction is specified, the robot goes to the specific coral
   * station, otherwise it just goes to the closest coral station.
   */
  public static class CoralStation extends PathDestination {
    Drive drive;
    Direction station;
    /**
     * Creates a Coral Station destination based on a specific direction.
     *
     * @param drive Drivetrain.
     * @param station Which Coral Station to path to, relative to the drivers' perspective. If None
     *     is chosen, the closest station is pathed to.
     */
    public CoralStation(Drive drive, Direction station) {
      this.station = station;
      this.drive = drive;
    }

    public CoralStation(Drive drive) {
      this.drive = drive;
      this.station = Direction.None;
    }

    @Override
    public Pose2d getTargetPosition() {
      switch (station) {
        //left coral station
        case Left:
          return allianceFlip(new Pose2d(1.56, 7.36, new Rotation2d(Degrees.of(-54))));
        //right coral station
        case Right:
          return allianceFlip(new Pose2d(1.623, 0.682, new Rotation2d(Degrees.of(54))));
        default:
        //closest coral station
          return drive
              .getPose()
              .nearest(
                  List.of(
                      allianceFlip(new Pose2d(1.56, 7.36, new Rotation2d(Degrees.of(-54)))),
                      allianceFlip(new Pose2d(1.623, 0.682, new Rotation2d(Degrees.of(54))))));
      }
    }
  }
  /** Reef destination. */
  public static class Reef extends PathDestination {
    Direction direction;
    int tagId;
    /**
     * Creates a reef direction based on the currently visible tag.
     *
     * @param direction Whether to path to the left branch or the right branch
     * @param tagId ID of the tag used for pathing.
     */
    public Reef(Direction direction, int tagId) {
      this.direction = direction;
      this.tagId = tagId;
    }

    @Override
    public Pose2d getTargetPosition() {
      // rotates the left or right pose around the reef based on the tag id
      return getReefPose(direction, tagId);
    }
    /**
     * Gets the pose of the left/right/center of the side of the reef with the tag id
     * @param direction 
     * @param targetTagId
     * @return Pose2d
     */
    public static Pose2d getReefPose(Direction direction, int targetTagId) {
      Pose2d o =
          switch (direction) {
            case Left -> DriveConstants.reefLeft;
            case Right -> DriveConstants.reefRight;
            case None -> DriveConstants.reefCenter;
          };
      Rotation2d rot =
          VisionConstants.aprilTagLayout.getTagPose(targetTagId).get().getRotation().toRotation2d();
        //180 degree rotation if the robot is flipped
      if (!isFlipped) rot = rot.plus(new Rotation2d(Math.PI));
      //rotates pose around reef center
      return allianceFlip(o.rotateAround(new Translation2d(4.5, 4.03), rot));
    }
    /**
     * Returns all the reef poses based on the side of each side of the reef
     * @param direction left/right/none(center)
     * @return ArrayList of poses
     */
    public static ArrayList<Pose2d> getReefPoses(Direction direction) {
      ArrayList<Pose2d> poses = new ArrayList<>();
      for (int i = 0; i < 6; i++) {
        poses.add(getReefPose(direction, i + (isFlipped ? 6 : 17)));
      }
      Pose2d[] p = new Pose2d[6];
      poses.toArray(p);
      return poses;
    }
  }

  /**
   * Aligns the robot to the reef based on closest reef side, and then paths to the reef based on direction (left/right/none)
   * Also turns led orange
   * @param drive
   * @param direction
   * @param controller
   * @param led
   * @return ParallelCommandGroup
   */
  public static Command reefAlign(
      Drive drive, Direction direction, CommandXboxController controller, LED led) {
    return new ParallelCommandGroup(
        led.turnColor(Color.kOrange),
        pathToDestination(
            drive,
            () ->
                new Reef(
                    direction,
                    //gets index of closest reef pose
                    Reef.getReefPoses(direction)
                            .indexOf(drive.getPose().nearest(Reef.getReefPoses(direction)))
                        + ((isFlipped) ? 6 : 17)),
            controller,
            direction,
            led));
  }

  /**
   * Raises elevator/arm and reefAligns simultaneously, then automatically scores coral in reef until lasercan no longer detects coral
   * @return ParallelCommandGroup w/ SequentialCommandGroup inside
   */
  public static Command reefScore(
      Drive drive,
      Direction direction,
      Level level,
      CommandXboxController controller,
      LED led,
      Elevator elevator,
      Arm arm,
      EndEffector endEffector) {
    return new ParallelCommandGroup(
        //raises elevator and moves end effector to preset position based on which preset
            (level == Level.L4)
                ? PresetCommands.presetL4(elevator, endEffector, arm)
                : (level == Level.L3)
                    ? PresetCommands.presetL3(elevator, endEffector, arm)
                    : PresetCommands.presetL2(elevator, endEffector, arm),
            reefAlign(drive, direction, controller, led))
        .andThen(
          //stops the preset command
            PresetCommands.stopAll(elevator, endEffector, arm),
            ((level == Level.L4)
            //shoots out -100 if L4 preset and shoots 100 if L2 or L3 preset
                ? endEffector.setEndEffectorVelocity(-100)
                : endEffector.setEndEffectorVelocity(100)))
        .until(
          //stops the shooting when both lasercans no longer detect the coral in the end effector
            () ->
                endEffector.getIO().getLaserCanMeasurement1() > 100
                    && endEffector.getIO().getLaserCanMeasurement2() > 100)
                    //sets end effector back to 0
        .andThen(() -> endEffector.setEndEffectorVelocity(0));
  }

  public enum Direction {
    Left,
    Right,
    None
  }

  public enum Level {
    L2,
    L3,
    L4
  }
}

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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveCommands.Direction;
import frc.robot.commands.PresetCommands;
import frc.robot.commands.SimCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmPresetConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPresetConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorConstants;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private SwerveDriveSimulation driveSimulation = null;
  public final Vision vision;
  public final LED led;

  private final Elevator elevator;

  private final EndEffector endEffector;

  private final Arm arm;

  private SimMechanismPoseHandler poseHandler;


  // Controllers
  public CommandXboxController driverController = new CommandXboxController(0);

  public CommandXboxController operatorController = new CommandXboxController(1);

  public CommandXboxController simController = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (pose) -> {});
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0));
        // new VisionIOPhotonVision(
        //     VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        endEffector = new EndEffector(new EndEffectorIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        arm = new Arm(new ArmIOSpark());
        led = new LED();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        led = new LED();
        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim(((ElevatorIOSim) elevator.getIO()).getLigamentEnd()));
        endEffector =
        new EndEffector(
            new EndEffectorIOSim(
                driveSimulation, () -> SimMechanismPoseHandler.getEndEffectorCoralSimPose(elevator, arm), this::stowed));
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});

        led = new LED();
        endEffector = new EndEffector(new EndEffectorIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        arm = new Arm(new ArmIO() {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up commands for auto
    NamedCommands.registerCommand("L4 Preset", PresetCommands.presetL4(elevator, endEffector, arm));
    NamedCommands.registerCommand("L2 Preset", PresetCommands.presetL2(elevator, endEffector, arm));

    NamedCommands.registerCommand("Endeffector Out", endEffector.setEndEffectorVelocity(100));
    NamedCommands.registerCommand("Endeffector Out L4", endEffector.setEndEffectorVelocity(-100));
    NamedCommands.registerCommand("Endeffector Stop", endEffector.setEndEffectorVelocity(0));
    NamedCommands.registerCommand("OuttakeLaserCan", PresetCommands.outtakeLaserCan(endEffector));
    NamedCommands.registerCommand(
        "Align Left", DriveCommands.reefAlign(drive, Direction.Left, driverController, led));
    NamedCommands.registerCommand(
        "Align Center", DriveCommands.reefAlign(drive, Direction.None, driverController, led));
    NamedCommands.registerCommand(
        "Align Right", DriveCommands.reefAlign(drive, Direction.Right, driverController, led));
    NamedCommands.registerCommand("IntakeLaserCAN", PresetCommands.intakeLaserCan(endEffector));
    NamedCommands.registerCommand(
        "Stop Preset", PresetCommands.stopAll(elevator, endEffector, arm));
    NamedCommands.registerCommand(
        "StowPreset", PresetCommands.stowElevator(elevator, endEffector, arm));

    NamedCommands.registerCommand(
        "Add Left Coral", SimCommands.addLeftCoral(driveSimulation::getSimulatedDriveTrainPose));
    NamedCommands.registerCommand(
        "AddRightCoral", SimCommands.addRightCoral(driveSimulation::getSimulatedDriveTrainPose));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    arm.addRoutinesToChooser(autoChooser);
    elevator.addRoutinesToChooser(autoChooser);
    endEffector.addRoutinesToChooser(autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // LED Stuff

    led.isCloseToBarge(drive)
        .and(() -> RobotState.isTeleop())
        .whileTrue(led.turnColor(Color.kWhite));
    led.isCriticalToBarge(drive)
        .and(() -> RobotState.isTeleop())
        .onTrue(led.sendBargeIndicator(operatorController))
        .whileTrue(led.turnColor(Color.kDarkBlue));
    new Trigger(() -> RobotState.isDisabled()).whileTrue(led.pulseBlue());
  }

  public static double processedJoystickInput(
      double controllerAxis, double deadband, double sensitivity, double scalar) {
    if (Math.abs(controllerAxis) < deadband) {
      return 0;
    } else {
      return Math.signum(controllerAxis)
          * Math.abs(
              scalar
                  * (1 / Math.pow((1.0 - deadband), sensitivity))
                  * Math.pow((Math.abs(controllerAxis) - deadband), sensitivity));
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive

    led.setDefaultCommand(led.goRainbow());

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () ->
                -driverController.getLeftY()
                    * Math.pow(Math.abs(driverController.getLeftY()), 1.2 - 1),
            () ->
                -driverController.getLeftX()
                    * Math.pow(Math.abs(driverController.getLeftX()), 1.2 - 1),
            () ->
                -driverController.getRightX()
                    * Math.pow(Math.abs(driverController.getRightX()), 2.2 - 1)));

    elevator.setDefaultCommand(
        elevator.setElevatorVelocity(
            () -> processedJoystickInput(-operatorController.getLeftY(), 0.2, 3.0, 25)));

    arm.setDefaultCommand(
        arm.setArmVelocity(
            () -> processedJoystickInput(-operatorController.getRightY(), 0.2, 1.0, 7 * Math.PI)));

    operatorController
        .leftTrigger()
        .whileTrue(endEffector.setEndEffectorVelocity(100))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    operatorController
        .rightTrigger()
        .whileTrue(endEffector.setEndEffectorVelocity(-100))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    operatorController.a().onTrue(PresetCommands.stowElevator(elevator, endEffector, arm));
    operatorController.b().whileTrue(PresetCommands.presetL2(elevator, endEffector, arm));
    operatorController.x().whileTrue(PresetCommands.presetL3(elevator, endEffector, arm));
    operatorController.y().whileTrue(PresetCommands.presetL4(elevator, endEffector, arm));

    operatorController
        .rightBumper()
        .onTrue(endEffector.setEndEffectorSpeed(-1))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    // operatorController.leftBumper().whileTrue(PresetCommands.netShoot(arm, endEffector));
    operatorController.povUp().onTrue(PresetCommands.outtakeLaserCan(endEffector));

    operatorController.leftBumper().onTrue(PresetCommands.intakeLaserCan(endEffector));

    driverController
        .rightBumper()
        .whileTrue(endEffector.setEndEffectorVelocity(60))
        .onFalse(endEffector.setEndEffectorVelocity(0));
    driverController
        .leftTrigger()
        .whileTrue(endEffector.setEndEffectorVelocity(-60))
        .onFalse(endEffector.setEndEffectorVelocity(0));
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.disableVision();
                    })
                .andThen(
                    Commands.runOnce(
                            () ->
                                drive.setPose(
                                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                            drive)
                        .ignoringDisable(true)));

    // Turns to tag and locks rotation
    // driverController
    //     .y()
    //     .whileTrue(
    //         DriveCommands.reefStrafe(
    //             drive, () -> driverController.getLeftY(), () -> driverController.getLeftX()));
    Command reefScoreLeftL3 =
        DriveCommands.reefScore(
            drive,
            Direction.Left,
            DriveCommands.Level.L3,
            driverController,
            led,
            elevator,
            arm,
            endEffector);
    Command reefAlignLeft = DriveCommands.reefAlign(drive, Direction.Left, driverController, led);
    Command reefAlignRight = DriveCommands.reefAlign(drive, Direction.Right, driverController, led);
    driverController
        .leftBumper()
        .and(drive::usingVision)
        .and(
            new Trigger(
                    () ->
                        !(reefScoreLeftL3.isScheduled()
                            || reefAlignLeft.isScheduled()
                            || reefAlignRight.isScheduled()))
                .debounce(0.5))
        .onTrue(reefScoreLeftL3.until(driverController.leftBumper().negate()));
    driverController
        .povLeft()
        .and(drive::usingVision)
        .and(
            new Trigger(
                    () ->
                        !(reefScoreLeftL3.isScheduled()
                            || reefAlignLeft.isScheduled()
                            || reefAlignRight.isScheduled()))
                .debounce(0.5))
        .onTrue(reefAlignLeft.until(driverController.povLeft().negate()));
    driverController
        .povRight()
        .and(drive::usingVision)
        .and(
            new Trigger(
                    () ->
                        !(reefScoreLeftL3.isScheduled()
                            || reefAlignLeft.isScheduled()
                            || reefAlignRight.isScheduled()))
                .debounce(0.5))
        .onTrue(reefAlignRight.until(driverController.povRight().negate()));

    driverController
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  DriveCommands.slowMode = 0.7;
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  DriveCommands.slowMode = 1;
                }));
    // simController.a().onTrue(SimulatedArena.getInstance().addGamePiece(new
    // ReefscapeCoralOnField(null)));
    if (Constants.currentMode == Mode.SIM) {
      simController
          .b()
          .onTrue(SimCommands.addLeftCoral(driveSimulation::getSimulatedDriveTrainPose));
      simController
          .x()
          .onTrue(SimCommands.addRightCoral(driveSimulation::getSimulatedDriveTrainPose));
    }
  }

  public boolean stowed() {
    return Math.abs(arm.getArmPosition() - ArmPresetConstants.ARM_STOW_FINAL)
            < ArmConstants.POSITION_TOLERANCE
        && Math.abs(elevator.getElevatorPosition() - ElevatorPresetConstants.ELEVATOR_STOW)
            < ElevatorConstants.POSITION_TOLERANCE;
  }

  public void periodic() {}

  public void teleopInit() {
    endEffector.setEndEffectorVelocity(0);
    elevator.setElevatorVelocity(() -> 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command stopMotors() {
    return PresetCommands.stopAll(elevator, endEffector, arm);
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}

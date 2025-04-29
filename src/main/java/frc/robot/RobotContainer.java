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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveCommands.Direction;
import frc.robot.commands.PresetCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  public final Vision vision;
  public final LED led;

  private final Elevator elevator;

  private final EndEffector endEffector;

  private final Arm arm;

  // Controllers
  public CommandXboxController driverController = new CommandXboxController(0);

  public CommandXboxController operatorController = new CommandXboxController(1);

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
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
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
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        led = new LED();
        endEffector = new EndEffector(new EndEffectorIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim(((ElevatorIOSim) elevator.getIO()).getLigamentEnd()));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        led = new LED();
        endEffector = new EndEffector(new EndEffectorIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        arm = new Arm(new ArmIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up commands for auto so it can be used in pathplanner
    NamedCommands.registerCommand("L4 Preset", PresetCommands.presetL4(elevator, endEffector, arm));
    NamedCommands.registerCommand("L2 Preset", PresetCommands.presetL2(elevator, endEffector, arm));

    NamedCommands.registerCommand("Endeffector Out", endEffector.setEndEffectorVelocity(100));
    NamedCommands.registerCommand("Endeffector Out L4", endEffector.setEndEffectorVelocity(-100));
    NamedCommands.registerCommand("Endeffector Stop", endEffector.setEndEffectorVelocity(0));
    NamedCommands.registerCommand("OutakeLaserCan", PresetCommands.outtakeLaserCan(endEffector));
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

    // Configure the button bindings
    configureButtonBindings();

    // LED check if it is close/critical to barge

    led.isCloseToBarge(drive)
        .and(() -> RobotState.isTeleop())
        .whileTrue(led.turnColor(Color.kWhite));
    led.isCriticalToBarge(drive)
        .and(() -> RobotState.isTeleop())
        .onTrue(led.sendBargeIndicator(operatorController))
        .whileTrue(led.turnColor(Color.kDarkBlue));
    new Trigger(() -> RobotState.isDisabled()).whileTrue(led.pulseBlue());
  }
  //deadband on controllers (anything less than 0.2 is not counted)
  public static double deadband(double controllerAxis) {
    if (Math.abs(controllerAxis) < 0.2) {
      return 0;
    } else {
      return (1 / (1 - 0.2)) * (controllerAxis + (Math.signum(controllerAxis) * 0.2));
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
        //default command is the joystick drive, with the suppliers being the left and right joysticks
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
        //default command for elevator is to get left joystick input from operator
        elevator.setElevatorVelocity(() -> deadband(-operatorController.getLeftY()) * 25));
    //default command for arm is to get right joystick input from operator
    arm.setDefaultCommand(arm.setArmVelocity(() -> 2 * Math.PI * -operatorController.getRightY()));

    //while operator presses left trigger, end effector goes out
    operatorController
        .leftTrigger()
        .whileTrue(endEffector.setEndEffectorVelocity(100))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    //while operator presses right trigger, end effector goes in
    operatorController
        .rightTrigger()
        .whileTrue(endEffector.setEndEffectorVelocity(-100))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    operatorController.a().onTrue(PresetCommands.stowElevator(elevator, endEffector, arm));
    //presets (cancels if operator lets go)
    operatorController.b().whileTrue(PresetCommands.presetL2(elevator, endEffector, arm));
    operatorController.x().whileTrue(PresetCommands.presetL3(elevator, endEffector, arm));
    operatorController.y().whileTrue(PresetCommands.presetL4(elevator, endEffector, arm));

    //weird I think does the same as the right trigger? setEndEffectorSpeed I think is like the same as velocity, but endEffectorSpeed caps at -1/1 (full speed)
    operatorController
        .rightBumper()
        .onTrue(endEffector.setEndEffectorSpeed(-1))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    // operatorController.leftBumper().whileTrue(PresetCommands.netShoot(arm, endEffector));
    operatorController.povUp().onTrue(PresetCommands.outtakeLaserCan(endEffector));

    operatorController.leftBumper().onTrue(PresetCommands.intakeLaserCan(endEffector));

    //driver can also shoot out l2,l3,intake
    driverController
        .rightBumper()
        .whileTrue(endEffector.setEndEffectorVelocity(60))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    //driver can also shoot out l4
    driverController
        .leftTrigger()
        .whileTrue(endEffector.setEndEffectorVelocity(-60))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    //driver presses b to reset gyro
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
        .and(() -> drive.useVision)
        .and(
            new Trigger(
                    () ->
                        !(reefScoreLeftL3.isScheduled()
                            || reefAlignLeft.isScheduled()
                            || reefAlignRight.isScheduled()))
                .debounce(0.5))
        .whileTrue(reefScoreLeftL3);
    driverController
        .povLeft()
        .and(() -> drive.useVision)
        .and(
            new Trigger(
                    () ->
                        !(reefScoreLeftL3.isScheduled()
                            || reefAlignLeft.isScheduled()
                            || reefAlignRight.isScheduled()))
                .debounce(0.5))
        .whileTrue(reefAlignLeft);
    driverController
        .povRight()
        .and(() -> drive.useVision)
        .and(
            new Trigger(
                    () ->
                        !(reefScoreLeftL3.isScheduled()
                            || reefAlignLeft.isScheduled()
                            || reefAlignRight.isScheduled()))
                .debounce(0.5))
        .whileTrue(reefAlignRight);

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
  }

  public void periodic() {}

  public void teleopInit() {
    //on teleop init, put laser can measurement to smartdashboard and set velocities to 0
    SmartDashboard.putNumber("Laser Can", endEffector.io.getLaserCanMeasurement1());
    endEffector.setEndEffectorVelocity(0);
    elevator.setElevatorVelocity(() -> 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        PresetCommands.stopAll(elevator, endEffector, arm), autoChooser.get());
  }
}

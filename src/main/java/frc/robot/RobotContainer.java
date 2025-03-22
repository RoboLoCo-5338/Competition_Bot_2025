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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveCommands.Direction;
import frc.robot.commands.EndEffectorCommands;
import frc.robot.commands.GroundIntakeCommands;
import frc.robot.commands.PresetCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSpark;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
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
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.groundintake.GroundIntakeIO;
import frc.robot.subsystems.groundintake.GroundIntakeIOSim;
import frc.robot.subsystems.groundintake.GroundIntakeIOSpark;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  public final Vision vision;

  public final LED led;

  private final Elevator elevator;

  private final GroundIntake groundIntake;

  private final EndEffector endEffector;

  private final ButtonBindings ButtonBindingsController;

  private final Climb climb;

  private final Arm arm;

  public CommandXboxController driverController = new CommandXboxController(0);

  public CommandXboxController operatorController = new CommandXboxController(1);

  // Controller

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Vision Target
  private int visionTargetID = -1;

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
        groundIntake = new GroundIntake(new GroundIntakeIOSpark());
        endEffector = new EndEffector(new EndEffectorIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        climb = new Climb(new ClimbIOTalonFX());
        arm = new Arm(new ArmIOSpark());
        led = new LED();

        //  led.setBargeIndicator(drive, elevator);
        ButtonBindingsController =
            new ButtonBindings(drive, led, elevator, groundIntake, endEffector, climb, arm);

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
        groundIntake = new GroundIntake(new GroundIntakeIOSim());
        endEffector = new EndEffector(new EndEffectorIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        climb = new Climb(new ClimbIOSim());
        arm = new Arm(new ArmIOSim(((ElevatorIOSim) elevator.getIO()).getLigamentEnd()));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        ButtonBindingsController =
            new ButtonBindings(drive, led, elevator, groundIntake, endEffector, climb, arm);
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
        groundIntake = new GroundIntake(new GroundIntakeIO() {});
        endEffector = new EndEffector(new EndEffectorIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        climb = new Climb(new ClimbIO() {});
        arm = new Arm(new ArmIO() {});
        ButtonBindingsController =
            new ButtonBindings(drive, led, elevator, groundIntake, endEffector, climb, arm);
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up commands for auto
    NamedCommands.registerCommand("GroundI Outake", groundIntake.setGroundIntakeVelocity(-3600));
    NamedCommands.registerCommand("GroundI Stop", groundIntake.setGroundIntakeVelocity(0));
    NamedCommands.registerCommand("L4 Preset", PresetCommands.presetL4(elevator, endEffector, arm));
    NamedCommands.registerCommand(
        "Endeffector Out",
        new RunCommand(() -> EndEffectorCommands.moveEndEffector(endEffector, 60), endEffector));
    NamedCommands.registerCommand(
        "Endeffector In", EndEffectorCommands.moveEndEffectorLaserCan(endEffector));
    NamedCommands.registerCommand(
        "Endeffector Stop", EndEffectorCommands.moveEndEffector(endEffector, 0));
    NamedCommands.registerCommand(
        "Align Left", DriveCommands.reefAlign(drive, Direction.Left, driverController, led));
    NamedCommands.registerCommand(
        "Align Right", DriveCommands.reefAlign(drive, Direction.Right, driverController, led));

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

    new SequentialCommandGroup(
            new WaitCommand(2.0),
            led.turnColor(Color.kWhite),
            new WaitCommand(0.3),
            led.turnOff(),
            new WaitCommand(0.3),
            led.turnColor(Color.kWhite),
            new WaitCommand(0.3),
            led.turnOff(),
            new WaitCommand(0.3),
            led.turnColor(Color.kWhite))
        .schedule(); // start it off as rainbow
  }

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

    led.isCloseToBarge(drive).whileTrue(led.setBargeIndicator(drive, elevator));
    elevator.setDefaultCommand(
        elevator.setElevatorVelocity(() -> deadband(-operatorController.getLeftY()) * 25));

    arm.setDefaultCommand(
        arm.setArmVelocity(
            () -> -operatorController.getRightY() * Math.abs(operatorController.getRightY())));

    operatorController
        .leftTrigger()
        .whileTrue(endEffector.setEndEffectorVelocity(100))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    operatorController
        .rightTrigger()
        .whileTrue(endEffector.setEndEffectorVelocity(-100))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    operatorController
        .leftBumper()
        .whileTrue(PresetCommands.netShoot(arm, endEffector))
        .onFalse(PresetCommands.stopAll(elevator, endEffector, arm));

    operatorController
        .a()
        .whileTrue(PresetCommands.stowElevator(elevator, endEffector, arm))
        .onFalse(PresetCommands.stopAll(elevator, endEffector, arm));
    operatorController
        .b()
        .whileTrue(PresetCommands.presetL2(elevator, endEffector, arm))
        .onFalse(PresetCommands.stopAll(elevator, endEffector, arm));
    operatorController
        .x()
        .whileTrue(PresetCommands.presetL3(elevator, endEffector, arm))
        .onFalse(PresetCommands.stopAll(elevator, endEffector, arm));
    operatorController
        .y()
        .whileTrue(PresetCommands.presetL4(elevator, endEffector, arm))
        .onFalse(PresetCommands.stopAll(elevator, endEffector, arm));

    operatorController
        .rightBumper()
        .onTrue(endEffector.setEndEffectorSpeed(-1))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    operatorController
        .leftBumper()
        .onTrue(PresetCommands.netShoot(arm, endEffector))
        .onFalse(PresetCommands.stopAll(elevator, endEffector, arm));

    operatorController
        .povUp()
        .whileTrue(GroundIntakeCommands.moveGroundIntakeArmRaw(groundIntake, -1.0))
        .onFalse(GroundIntakeCommands.moveGroundIntakeArmRaw(groundIntake, 0.0));

    operatorController
        .povDown()
        .whileTrue(GroundIntakeCommands.moveGroundIntakeArmRaw(groundIntake, 1.0))
        .onFalse(GroundIntakeCommands.moveGroundIntakeArmRaw(groundIntake, 0.0));

    operatorController
        .povRight()
        .whileTrue(GroundIntakeCommands.moveGroundIntakeWheels(groundIntake, 1.0))
        .onFalse(GroundIntakeCommands.moveGroundIntakeWheels(groundIntake, 0.0));

    operatorController
        .povLeft()
        .whileTrue(GroundIntakeCommands.moveGroundIntakeWheels(groundIntake, -1.0))
        .onFalse(GroundIntakeCommands.moveGroundIntakeWheels(groundIntake, 0.0));

    driverController
        .rightBumper()
        .whileTrue(endEffector.setEndEffectorVelocity(60))
        .onFalse(endEffector.setEndEffectorVelocity(0));
    driverController
        .leftTrigger()
        .whileTrue(endEffector.setEndEffectorVelocity(-60))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    driverController
        .x()
        .onTrue(groundIntake.setIntakeSpeed(-1))
        .onFalse(groundIntake.setGroundIntakeVelocity(0));
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      System.out.println("runs");
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
    driverController
        .povLeft()
        .and(() -> drive.useVision)
        .onTrue(DriveCommands.reefAlign(drive, Direction.Left, driverController, led));
    driverController
        .povRight()
        .and(
            () -> {
              System.out.println(
                  "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb" + drive.useVision);
              return drive.useVision;
            })
        .onTrue(DriveCommands.reefAlign(drive, Direction.Right, driverController, led));

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

  public void periodic() {
    // ButtonBindingsController.periodic();
    Logger.recordOutput("camera pose", Constants.VisionConstants.robotToCamera0);
    // System.out.println(drive.useVision);
  }

  public void teleopInit() {
    SmartDashboard.putNumber("Laser Can", endEffector.io.getLaserCanmeasurement1());
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

  public void setVisionTarget(int id) {
    visionTargetID = id;
  }
}

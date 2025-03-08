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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
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
import frc.robot.subsystems.led.AddressableLEDIO;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.led.LEDIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final LED led;

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
        led = new LED(new AddressableLEDIO());
        groundIntake = new GroundIntake(new GroundIntakeIOSpark());
        endEffector = new EndEffector(new EndEffectorIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        climb = new Climb(new ClimbIOTalonFX());
        arm = new Arm(new ArmIOSpark());
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
        led = new LED(new LEDIOSim());
        groundIntake = new GroundIntake(new GroundIntakeIOSim());
        endEffector = new EndEffector(new EndEffectorIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        climb = new Climb(new ClimbIOSim());
        arm = new Arm(new ArmIOSim(((ElevatorIOSim) elevator.getIO()).getLigamentEnd()));
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

        led = new LED(new LEDIO() {});
        groundIntake = new GroundIntake(new GroundIntakeIO() {});
        endEffector = new EndEffector(new EndEffectorIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        climb = new Climb(new ClimbIO() {});
        arm = new Arm(new ArmIO() {});
        ButtonBindingsController =
            new ButtonBindings(drive, led, elevator, groundIntake, endEffector, climb, arm);
        break;
    }

    NamedCommands.registerCommand("GroundI Outake", groundIntake.setGroundIntakeVelocity(-3600));
    NamedCommands.registerCommand("GroundI Stop", groundIntake.setGroundIntakeVelocity(0));

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
    // autoChooser.addOption(
    //     "1m Forward", A);

    // Configure the button bindings
    configureButtonBindings();
  }

  private double deadband(double controllerAxis) {
    if (Math.abs(controllerAxis) < 0.2) {
      return 0;
    } else {
      return (1 / (1 - 0.2)) * (controllerAxis + (Math.signum(controllerAxis) * 0.2));
    }
  }

  private double easyDeadband(double val) {
    if (Math.abs(val) < 0.07) {
      return 0;
    } else {
      return val;
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

    elevator.setDefaultCommand(
        elevator.setElevatorVelocity(() -> deadband(-operatorController.getLeftY()) * 25));

    arm.setDefaultCommand(
        arm.setArmVelocity(
            () -> -operatorController.getRightY() * Math.abs(operatorController.getRightY())));

    operatorController
        .leftTrigger()
        .whileTrue(endEffector.setEndEffectorVelocity(60))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    operatorController
        .rightTrigger()
        .whileTrue(endEffector.setEndEffectorVelocity(-60))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    operatorController.y().onTrue(arm.setArmPosition(0.2)).onFalse(arm.setArmVelocity(() -> 0));

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

    // operatorController
    //     .rightBumper()
    //     .onTrue(arm.setArmPosition(0.1))
    //     .onFalse(arm.setArmVelocity(() -> 0.0));

    // left trigger intake preset + intake rollers
    // left bumper outtake preset
    // right bumper stow
    // right trigger outtake
    driverController
        .rightTrigger()
        .onTrue(groundIntake.setGroundIntakeVelocity(3600))
        .onFalse(groundIntake.setGroundIntakeVelocity(0));

    driverController
        .leftTrigger()
        .onTrue(groundIntake.setGroundIntakeVelocity(-3600))
        .onFalse(groundIntake.setGroundIntakeVelocity(0));

    driverController
        .leftBumper()
        .onTrue(groundIntake.setGroundArmVelocity(() -> 10))
        .onFalse(groundIntake.setGroundArmVelocity(() -> 0.0));

    driverController
        .rightBumper()
        .onTrue(groundIntake.setGroundArmVelocity(() -> -10))
        .onFalse(groundIntake.setGroundArmVelocity(() -> 0.0));
    driverController
        .rightTrigger()
        .onTrue(groundIntake.setGroundIntakeVelocity(3600))
        .onFalse(groundIntake.setGroundIntakeVelocity(0.0));
    // operatorController
    //     .b()
    //     .whileTrue(elevator.setElevatorPosition(102))
    //     .onFalse(elevator.setElevatorVelocity(() -> 0.0));

    // operatorController
    //     .y()
    //     .whileTrue(elevator.setElevatorPosition(93))
    //     .onFalse(elevator.setElevatorVelocity(() -> 0.0));
    // // driverController

    //     .povUp()
    //     .and(driverController.x())
    //     .onTrue(climb.setClimbVelocity(0.4))
    //     .onFalse(climb.setClimbVelocity(0));
    driverController.povDown().onTrue(climb.setClimbVelocity(-2.9)).onFalse(climb.stopMotor());

    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  public void periodic() {
    ButtonBindingsController.periodic();
  }

  public void teleopInit() {
    SmartDashboard.putNumber("Laser Can", elevator.io.getLaserCanMeasurement());
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

  /**
   * Maps a joystick input to a speed using an exponential function, which gives more precise
   * control at lower speeds.
   *
   * @param x the input from the joystick
   * @return the output speed
   */
}

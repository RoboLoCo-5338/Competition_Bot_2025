package frc.robot.opponentsimulation.ussrivets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SimMechanismPoseHandler;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveCommands.Direction;
import frc.robot.commands.PresetCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.opponentsimulation.OpponentRobot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmPresetConstants;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPresetConstants;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class USSRivets extends OpponentRobot {
  private final Drive drive;
  private final Arm arm;
  private final Elevator elevator;
  private final EndEffector endEffector;
  private final LED led;
  private final Vision vision;
  private final SwerveDriveSimulation driveSimulation;
  private final CommandXboxController driverController, operatorController;
  private SimMechanismPoseHandler poseHandler;
  private double last_roller_position = 0.0;
  private double time_elapsed = 0.0;

  public USSRivets(
      int id, Alliance alliance, int driverControllerPort, int operatorControllerPort) {
    super(id, Drive.mapleSimConfig, alliance);

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
    elevator = new Elevator(new ElevatorIOSim());
    arm = new Arm(new ArmIOSim(((ElevatorIOSim) elevator.getIO()).getLigamentEnd()));
    endEffector =
        new EndEffector(
            new EndEffectorIOSim(
                driveSimulation,
                () -> SimMechanismPoseHandler.getEndEffectorCoralSimPose(elevator, arm),
                this::stowed));
    vision =
        new Vision(
            drive,
            new VisionIOPhotonVisionSim(
                VisionConstants.camera0Name,
                VisionConstants.robotToCamera0,
                driveSimulation::getSimulatedDriveTrainPose));
    led = new LED();
    driverController = new CommandXboxController(driverControllerPort);
    operatorController = new CommandXboxController(operatorControllerPort);
  }

  public boolean stowed() {
    return Math.abs(arm.getArmPosition() - ArmPresetConstants.ARM_STOW_FINAL)
            < ArmConstants.POSITION_TOLERANCE
        && Math.abs(elevator.getElevatorPosition() - ElevatorPresetConstants.ELEVATOR_STOW)
            < ElevatorConstants.POSITION_TOLERANCE;
  }

  @Override
  public void configureButtonBindings() {
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

    manualTrigger(operatorController.leftTrigger())
        .whileTrue(endEffector.setEndEffectorVelocity(100))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    manualTrigger(operatorController.rightTrigger())
        .whileTrue(endEffector.setEndEffectorVelocity(-100))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    manualTrigger(operatorController.a())
        .onTrue(PresetCommands.stowElevator(elevator, endEffector, arm));
    manualTrigger(operatorController.b())
        .whileTrue(PresetCommands.presetL2(elevator, endEffector, arm));
    manualTrigger(operatorController.x())
        .whileTrue(PresetCommands.presetL3(elevator, endEffector, arm));
    manualTrigger(operatorController.y())
        .whileTrue(PresetCommands.presetL4(elevator, endEffector, arm));

    manualTrigger(operatorController.rightBumper())
        .onTrue(endEffector.setEndEffectorSpeed(-1))
        .onFalse(endEffector.setEndEffectorVelocity(0));

    // manualTrigger( operatorController.leftBumper()).whileTrue(PresetCommands.netShoot(arm,
    // endEffector));
    manualTrigger(operatorController.povUp()).onTrue(PresetCommands.outtakeLaserCan(endEffector));

    manualTrigger(operatorController.leftBumper())
        .onTrue(PresetCommands.intakeLaserCan(endEffector));

    manualTrigger(driverController.rightBumper())
        .whileTrue(endEffector.setEndEffectorVelocity(60))
        .onFalse(endEffector.setEndEffectorVelocity(0));
    manualTrigger(driverController.leftTrigger())
        .whileTrue(endEffector.setEndEffectorVelocity(-60))
        .onFalse(endEffector.setEndEffectorVelocity(0));
    manualTrigger(driverController.b())
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
    //             drive, () ->manualTrigger( driverController.getLeftY()), () ->manualTrigger(
    // driverController.getLeftX())));
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
    manualTrigger(driverController.leftBumper())
        .and(drive::usingVision)
        .and(
            new Trigger(
                    () ->
                        !(reefScoreLeftL3.isScheduled()
                            || reefAlignLeft.isScheduled()
                            || reefAlignRight.isScheduled()))
                .debounce(0.5))
        .onTrue(reefScoreLeftL3.until(driverController.leftBumper().negate()));
    manualTrigger(driverController.povLeft())
        .and(drive::usingVision)
        .and(
            new Trigger(
                    () ->
                        !(reefScoreLeftL3.isScheduled()
                            || reefAlignLeft.isScheduled()
                            || reefAlignRight.isScheduled()))
                .debounce(0.5))
        .onTrue(reefAlignLeft.until(driverController.povLeft().negate()));
    manualTrigger(driverController.povRight())
        .and(drive::usingVision)
        .and(
            new Trigger(
                    () ->
                        !(reefScoreLeftL3.isScheduled()
                            || reefAlignLeft.isScheduled()
                            || reefAlignRight.isScheduled()))
                .debounce(0.5))
        .onTrue(reefAlignRight.until(driverController.povRight().negate()));

    manualTrigger(driverController.rightTrigger())
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
    // manualTrigger( simController.a()).onTrue(SimulatedArena.getInstance().addGamePiece(new
    // ReefscapeCoralOnField(null)));
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
}

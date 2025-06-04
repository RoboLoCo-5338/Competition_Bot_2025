package frc.robot.opponentsimulation.ussrivets;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.SimMechanismPoseHandler;
import frc.robot.generated.TunerConstants;
import frc.robot.opponentsimulation.OpponentRobot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmPresetConstants;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPresetConstants;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorConstants;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

public class USSRivets extends OpponentRobot {
  private final Drive drive;
  private final Arm arm;
  private final Elevator elevator;
  private final EndEffector endEffector;
  private final Vision vision;
  private final SwerveDriveSimulation driveSimulation;
  private SimMechanismPoseHandler poseHandler;
  private double last_roller_position = 0.0;
  private double time_elapsed = 0.0;

  public USSRivets(int id, Alliance alliance) {
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
           driveSimulation, () -> SimMechanismPoseHandler.getEndEffectorCoralSimPose(elevator, arm), this::stowed));
   vision =
       new Vision(
           drive,
           new VisionIOPhotonVisionSim(
               VisionConstants.camera0Name,
               VisionConstants.robotToCamera0,
               driveSimulation::getSimulatedDriveTrainPose));

  }

  public boolean stowed() {
    return Math.abs(arm.getArmPosition() - ArmPresetConstants.ARM_STOW_FINAL)
            < ArmConstants.POSITION_TOLERANCE
        && Math.abs(elevator.getElevatorPosition() - ElevatorPresetConstants.ELEVATOR_STOW)
            < ElevatorConstants.POSITION_TOLERANCE;
  }

  @Override
  public void configureButtonBindings() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'configureButtonBindings'");
  }
}

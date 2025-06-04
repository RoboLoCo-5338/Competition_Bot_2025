package frc.robot.opponentsimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public abstract class OpponentRobot {
  /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
  public static final Pose2d[] ROBOT_QUEENING_POSITIONS =
      new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d())
      };

  private final SelfControlledSwerveDriveSimulation driveSimulation;
  private final Pose2d queeningPose;
  private final int id;
  private final Mode mode;
  public final Alliance alliance;

  public OpponentRobot(int id, DriveTrainSimulationConfig config, Alliance alliance) {
    this.id = id;
    this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
    this.driveSimulation =
        new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(config, queeningPose));

    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());

    this.alliance = alliance;
    this.mode = Mode.Queening;
  }

  public enum Mode {
    Queening,
    Pathing,
    Manual,
    AI
  }

  public abstract void configureButtonBindings();
  public Trigger manualTrigger(BooleanSupplier condition) {
    return new Trigger(condition).and(() -> mode==Mode.Manual);
  }
}

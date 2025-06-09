package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

public class SimCommands {
  public static Command simCommand(Runnable runnable, Subsystem... subsystems) {
    return new InstantCommand(runnable, subsystems).onlyIf(() -> RobotBase.isSimulation());
  }

  public static Command addLeftCoral() {
    return simCommand(
        () ->
            SimulatedArena.getInstance()
                .addGamePiece(
                    new ReefscapeCoralOnField(
                        allianceFlip(new Pose2d(1.236, 7.121, Rotation2d.fromDegrees(-53.393))))));
  }

  public static Command addLeftCoral(Supplier<Pose2d> robotPose) {
    return simCommand(
        () -> {
          Pose2d curPose = allianceFlip(robotPose.get());
          if (Math.abs((curPose.getX() * 0.744908 + 6.15898) - curPose.getY()) < 0.25
              && Math.abs(robotPose.get().getRotation().getDegrees() + 54.462) < 5) {
            SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(robotPose.get()));
          } else
            SimulatedArena.getInstance()
                .addGamePiece(
                    new ReefscapeCoralOnField(
                        allianceFlip(new Pose2d(1.236, 7.121, Rotation2d.fromDegrees(-53.393)))));
        });
  }

  public static Command addRightCoral() {
    return simCommand(
        () -> {
          SimulatedArena.getInstance()
              .addGamePiece(
                  new ReefscapeCoralOnField(
                      allianceFlip(new Pose2d(1.217, 0.929, Rotation2d.fromDegrees(57.995)))));
        });
  }

  // TODO: maybe refactor this?
  public static Command addRightCoral(Supplier<Pose2d> robotPose) {
    return simCommand(
        () -> { // Checks the residual
          Pose2d curPose = allianceFlip(robotPose.get());
          if (Math.abs((curPose.getX() * -0.747624 + 1.85966) - curPose.getY()) < 0.25
              && Math.abs(robotPose.get().getRotation().getDegrees() - 53.807) < 5)
            SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(robotPose.get()));
          else
            SimulatedArena.getInstance()
                .addGamePiece(
                    new ReefscapeCoralOnField(
                        allianceFlip(new Pose2d(1.217, 0.929, Rotation2d.fromDegrees(57.995)))));
        });
  }

  public static Command addRobotCoral(Supplier<Pose2d> robotPose) {
    return simCommand(
        () ->
            SimulatedArena.getInstance()
                .addGamePiece(new ReefscapeCoralOnField(allianceFlip(robotPose.get()))));
  }

  private static Pose2d allianceFlip(Pose2d pose) {
    return (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue))
        ? pose
        : FlippingUtil.flipFieldPose(pose);
  }
}

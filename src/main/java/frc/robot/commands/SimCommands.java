package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

public class SimCommands {
  public static Command addLeftCoral() {
    return new InstantCommand(
            () ->
                SimulatedArena.getInstance()
                    .addGamePiece(
                        new ReefscapeCoralOnField(
                            (DriverStation.getAlliance()
                                    .orElse(Alliance.Blue)
                                    .equals(Alliance.Blue))
                                ? new Pose2d(1.236, 1.236, Rotation2d.fromDegrees(-53.393))
                                : FlippingUtil.flipFieldPose(
                                    new Pose2d(1.236, 1.236, Rotation2d.fromDegrees(-53.393))))))
        .onlyIf(() -> RobotBase.isSimulation());
  }

  public static Command addRightCoral() {
    return new InstantCommand(
            () -> {
              System.out.println("runs");
              System.out.println(SimulatedArena.getInstance().getGamePiecesByType("Coral"));
              SimulatedArena.getInstance()
                  .addGamePiece(
                      new ReefscapeCoralOnField(
                          (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue))
                              ? new Pose2d(1.217, 0.929, Rotation2d.fromDegrees(57.995))
                              : FlippingUtil.flipFieldPose(
                                  new Pose2d(1.217, 0.929, Rotation2d.fromDegrees(57.995)))));
              System.out.println(SimulatedArena.getInstance().getGamePiecesByType("Coral"));
            })
        .onlyIf(() -> RobotBase.isSimulation());
  }
}

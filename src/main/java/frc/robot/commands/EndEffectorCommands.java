package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.endeffector.*;

public class EndEffectorCommands {
  public static Command moveEndEffector(EndEffector endEffector, double speed) {
    return new InstantCommand(() -> endEffector.setEndEffectorVelocity(speed));
  }
  // value is in rotations
  public static double getEndEffectorPosition(EndEffector endEffector) {
    return endEffector.io.endEffectorMotor.getPosition().getValueAsDouble();
  }

    public static Command moveEndEffectorLaserCan(EndEffector endEffector) {
    // return new SequentialCommandGroup(
    //     endEffector.setEndEffectorVelocity(60),
    //     new WaitUntilCommand(
    //         () ->
    //             (endEffector.getIO().getLaserCanmeasurement1() > 101.6
    //                 && endEffector.getIO().getLaserCanMeasurement2() > 101.6)),
    //     endEffector.setEndEffectorSpeed(0));
    return new RunCommand(() -> endEffector.setEndEffectorVelocity(60), endEffector)
        .until(
            () ->
                (endEffector.getIO().getLaserCanmeasurement1() > 100
                    && endEffector.getIO().getLaserCanMeasurement2() > 100));
  }
}

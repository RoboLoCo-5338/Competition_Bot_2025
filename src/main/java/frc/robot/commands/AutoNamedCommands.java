package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.endeffector.EndEffector;

public class AutoNamedCommands {
  public static Command moveEndEffectorLaserCan(EndEffector endEffector) {
    System.out.println("Moving end effector");
    if (endEffector.getIO().getLaserCanMeasurement1() == -1
        || endEffector.getIO().getLaserCanMeasurement2() == -1) {
      System.out.println("At least one LaserCAN measurement is broken");
      return new InstantCommand();
    }
    return new SequentialCommandGroup(
        new InstantCommand(() -> System.out.println("0")),
        new RepeatCommand(endEffector.setEndEffectorVelocity(60))
            .onlyWhile(
                () ->
                    (endEffector.getIO().getLaserCanMeasurement1() > 100
                        && endEffector.getIO().getLaserCanMeasurement2() > 100)),
        new InstantCommand(() -> System.out.println("1")),
        endEffector.setEndEffectorVelocity(0.0),
        new InstantCommand(() -> System.out.println("2")));
  }

  public static Command laserCanOutake(EndEffector endEffector) {
    if (endEffector.getIO().getLaserCanMeasurement1() == -1
    || endEffector.getIO().getLaserCanMeasurement2() == -1) {
    System.out.println("At least one LaserCAN measurement is broken");
    return new InstantCommand();
  } 
    return new SequentialCommandGroup(
        new RepeatCommand(endEffector.setEndEffectorVelocity(60))
            .onlyWhile(
                () -> (endEffector.getIO().getLaserCanMeasurement1() > 100
                && endEffector.getIO().getLaserCanMeasurement2() < 100)),
        endEffector.setEndEffectorVelocity(0)
  );
}
}

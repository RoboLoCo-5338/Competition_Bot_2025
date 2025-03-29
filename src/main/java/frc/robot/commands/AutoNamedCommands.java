package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.PresetConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;

public class AutoNamedCommands {
    public static Command fullIn(Elevator elevator, EndEffector endEffector, Arm arm) {
        return new SequentialCommandGroup(
            arm.setArmPosition(0.61), new WaitCommand(0.3), elevator.setElevatorPosition(0.05));
      }
    
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
                        (endEffector.getIO().getLaserCanMeasurement1() < 100
                            && endEffector.getIO().getLaserCanMeasurement2() < 100)),
            new InstantCommand(() -> System.out.println("1")),
            endEffector.setEndEffectorVelocity(0.0),
            new InstantCommand(() -> System.out.println("2")));
      }
}

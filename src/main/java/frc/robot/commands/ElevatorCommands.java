package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {

  public static Command setElevator(Elevator elevator, double position) {

    return elevator.setElevatorPosition(position);
  }

  public static Command moveElevator(Elevator elevator, double speed) {

    return elevator.setElevatorVelocity(speed);
  }

  public static double getElevatorPosition(Elevator elevator) {
    return elevator.io
        .getLaserCanMeasurement(); // actual position of the elevator, NOT the setpoint
  }
}

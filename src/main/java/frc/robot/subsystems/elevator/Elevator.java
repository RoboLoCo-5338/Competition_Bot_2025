package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Mode;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double prevError = 0;
  private double integral = 0;
  private double error;
  private final Alert elevator1DisconnectedAlert =
      new Alert("Elevator motor 1 disconnected", AlertType.kError);
  private final Alert elevator2DisconnectedAlert =
      new Alert("Elevator motor 1 disconnected", AlertType.kError);

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  /**
   * Periodic function for the Elevator subsystem. Updates inputs and logs them. Checks if the
   * elevator motors are connected and alerts if not.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    elevator1DisconnectedAlert.set(!inputs.elevator1Connected && Constants.currentMode != Mode.SIM);
    elevator2DisconnectedAlert.set(!inputs.elevator1Connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Creates a command to set the elevator's position to the specified value.
   *
   * @param position The desired position for the elevator.
   * @return A command that sets the elevator's position.
   */
  public Command setElevatorPosition(double position) {
    return new RunCommand(() -> io.setElevatorPosition(position), this);
  }

  /**
   * PID controller for the elevator. This function uses the laser can to measure the elevator's
   * position and calculates the error, integral, and derivative of the error. It then applies the
   * gains to the error, integral, and derivative to calculate the output velocity. The output
   * velocity is then set on the elevator.
   *
   * @param position The desired position of the elevator.
   */
  private void elevatorPID(double position) {
    double curPosition = io.getLaserCanMeasurement();
    if (curPosition == -1) {
      io.setElevatorVelocity(0);
      return;
    }
    error = position - curPosition;
    integral += error;
    double derivative = error - prevError;
    double output =
        ElevatorConstants.ELEVATOR_kP_LASERCAN * error
            + ElevatorConstants.ELEVATOR_kI_LASERCAN * integral
            + ElevatorConstants.ELEVATOR_kD_LASERCAN * derivative;

    io.setElevatorVelocity(output);
  }

  public Command moveElevatorLaserCan(double position) {
    return new FunctionalCommand(
        () -> {
          integral = 0;
          prevError = 0;
          error = 0;
          io.setElevatorVelocity(0.0);
        },
        () -> elevatorPID(position),
        (interrupted) -> io.setElevatorVelocity(0.0),
        () -> Math.abs(error) < ElevatorConstants.ELEVATOR_EPSILON,
        this);
  }

  public Command setElevatorVelocity(DoubleSupplier velocity) {
    return new InstantCommand(() -> io.setElevatorVelocity(velocity.getAsDouble()), this);
  }

  public ElevatorIO getIO() {
    return io;
  }

  public void updatePID() {
    io.updatePID();
  }
}

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SysIDSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SysIDSubsystem<ElevatorIO, ElevatorIOInputsAutoLogged> {
  private final Alert elevator2DisconnectedAlert =
      new Alert("Elevator motor 1 disconnected", AlertType.kError);

  public Elevator(ElevatorIO io) {
    super(
        io,
        new ElevatorIOInputsAutoLogged(),
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())));
  }

  /**
   * Periodic function for the Elevator subsystem. Updates inputs and logs them. Checks if the
   * elevator motors are connected and alerts if not.
   */
  @Override
  public void periodic() {
    super.periodic();
    elevator2DisconnectedAlert.set(!inputs.elevator1Connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Creates a command to set the elevator's position to the specified value.
   *
   * @param position The desired position for the elevator.
   * @return A command that sets the elevator's position.
   */
  public Command setElevatorPosition(double position, int slot) {
    return new StartEndCommand(
            () -> io.setElevatorPosition(position, slot), () -> io.setElevatorVelocity(0), this)
        .until(
            () ->
                Math.abs(position - inputs.elevator1Position)
                    < ElevatorConstants.POSITION_TOLERANCE);
  }

  public Command setElevatorVelocity(DoubleSupplier velocity) {
    return new InstantCommand(() -> io.setElevatorVelocity(velocity.getAsDouble()), this);
  }

  public double getElevatorPosition() {
    return inputs.position;
  }

  @Override
  public String getName() {
    return "Elevator";
  }
}

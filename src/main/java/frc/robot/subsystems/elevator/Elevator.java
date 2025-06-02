package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SysIDSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase implements SysIDSubsystem {
  public final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double prevError = 0;
  private double integral = 0;
  private double error;
  private final Alert elevator1DisconnectedAlert =
      new Alert("Elevator motor 1 disconnected", AlertType.kError);
  private final Alert elevator2DisconnectedAlert =
      new Alert("Elevator motor 1 disconnected", AlertType.kError);

  private final SysIdRoutine sysIdRoutine;
  private boolean elevatorPositionRunning = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new Mechanism(io::elevatorOpenLoop, null, this));
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
  public Command setElevatorPosition(double position, int slot) {
    return new StartEndCommand(
            () -> {
              io.setElevatorPosition(position, slot);
              elevatorPositionRunning = true;
            },
            () -> {
              io.setElevatorVelocity(0);
              elevatorPositionRunning = false;
            },
            this)
        .until(
            new Trigger(() -> Math.abs(inputs.elevator1Velocity) < 0.001)
                .and(() -> elevatorPositionRunning)
                .debounce(0.5)
                .onTrue(
                    new InstantCommand()) // Why the heck does this need to be here? The code breaks
                // if it's not there.
                .or(
                    () ->
                        Math.abs(position - inputs.elevator1Position)
                            < ElevatorConstants.POSITION_TOLERANCE));
  }

  public Command setElevatorVelocity(DoubleSupplier velocity) {
    return new InstantCommand(() -> io.setElevatorVelocity(velocity.getAsDouble()), this);
  }

  public ElevatorIO getIO() {
    return io;
  }

  public double getElevatorPosition() {
    return inputs.elevator1Position;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }

  @Override
  public String getName() {
    return "Elevator ";
  }
}

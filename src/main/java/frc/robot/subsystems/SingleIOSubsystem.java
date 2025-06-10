package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import frc.robot.subsystems.IO.IOInputs;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class SingleIOSubsystem<T extends IO<I>, I extends IOInputs & LoggableInputs>
    extends AdvantageScopeSubsystem<T, I> {
  protected T io;
  public I input;

  public SingleIOSubsystem(T io, I inputs) {
    super(List.of(io), List.of(inputs));
    this.io = io;
    this.input = inputs;
  }

  @Override
  public void periodic() {
    io.updateInputs(input);
    Logger.processInputs(getName(), input);
    disconnectedAlerts.get(0).set(!input.connected);
  }

  @Override
  protected void addDisconnectedAlerts() {
    disconnectedAlerts.add(new Alert(getName() + " is disconnected.", Alert.AlertType.kError));
  }

  public T getIO() {
    return io;
  }
}

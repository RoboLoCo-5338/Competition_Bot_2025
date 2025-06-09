package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IO.IOInputs;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class AdvantageScopeSubsystem<T extends IO<I>, I extends IOInputs & LoggableInputs>
    extends SubsystemBase implements AutoCloseable {
  protected final List<T> ios;
  protected final List<I> inputs;
  protected final List<Alert> disconnectedAlerts;

  public AdvantageScopeSubsystem(List<T> ios, List<I> inputs) {
    this.ios = ios;
    this.inputs = inputs;
    this.disconnectedAlerts = new ArrayList<>(ios.size());
    addDisconnectedAlerts();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < ios.size(); i++) {
      ios.get(i).updateInputs(inputs.get(i));
      Logger.processInputs(getName() + "/Device" + i, inputs.get(i));
    }
    for (int i = 0; i < ios.size(); i++) {
      disconnectedAlerts.get(i).set(!inputs.get(i).connected);
    }
  }

  @Override
  public void close() {
    for (T device : ios) {
      device.close();
    }
  }

  public List<T> getIOs() {
    return ios;
  }

  protected void addDisconnectedAlerts() {
    for (int i = 0; i < ios.size(); i++) {
      disconnectedAlerts.add(
          new Alert(
              "Device " + i + " of Subsystem" + getName() + " is disconnected.",
              Alert.AlertType.kError));
    }
  }
}

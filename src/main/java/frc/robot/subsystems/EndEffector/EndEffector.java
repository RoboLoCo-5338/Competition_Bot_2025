package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  private final Alert endEffectorDisconnectedAlert =
      new Alert(" End Effector motor disconnected!!", AlertType.kError);

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("End Effector", inputs);

    endEffectorDisconnectedAlert.set(
        !inputs.endEffectorConnected && Constants.currentMode != Mode.SIM);
  }

  public Command setEndEffectorVelocity(double velocity) {
    return new InstantCommand(() -> io.setEndEffectorVelocity(velocity));
  }
}

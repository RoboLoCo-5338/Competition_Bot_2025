package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  //new input/output w/ motor
  public final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  //alert for when the end effector is disconnected
  private final Alert endEffectorDisconnectedAlert =
      new Alert(" End Effector motor disconnected!!", AlertType.kError);
  //sets io during object creation
  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    //periodically updates inputs (autologger)
    io.updateInputs(inputs);
    Logger.processInputs("End Effector", inputs);
    //sets alert if needed
    endEffectorDisconnectedAlert.set(
        !inputs.endEffectorConnected && Constants.currentMode != Mode.SIM);
  }
  /** instant command to set end effector velocity w/ subsystem requirement of the end effector */
  public Command setEndEffectorVelocity(double velocity) {
    return new InstantCommand(
        () -> {
          io.setEndEffectorVelocity(velocity);
        },
        this);
  }
  /**instant command to set end effector speed */
  public Command setEndEffectorSpeed(double speed) {
    return new InstantCommand(() -> io.setEndEffectorSpeed(speed));
  }

  public EndEffectorIO getIO() {
    return io;
  }
}

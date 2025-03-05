package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  public final ArmIO io;
  public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final Alert armDisconnectedAlert =
      new Alert("Arm motor disconnected", AlertType.kWarning);

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    armDisconnectedAlert.set(!inputs.armConnected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Sets the arm to the given position in degrees.
   *
   * <p>This is a blocking call and will wait until the arm is at the requested position.
   *
   * @param position The position to set the arm to in degrees.
   * @return A command that sets the arm to the given position.
   */
  public Command setArmPosition(double position) {
    return new InstantCommand(() -> io.setArmPosition(position), this);
  }

  /**
   * Sets the arm to the given velocity in degrees per second.
   *
   * <p>This is a non-blocking call and will not wait until the arm is at the requested velocity.
   *
   * @param velocity The velocity to set the arm to in degrees per second.
   * @return A command that sets the arm to the given velocity.
   */
  public Command setArmVelocity(double velocity) {
    return new InstantCommand(() -> io.setArmVelocity(velocity), this);
  }
}

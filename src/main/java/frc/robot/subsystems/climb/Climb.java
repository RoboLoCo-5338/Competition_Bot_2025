package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

  public final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private final Alert climbMotorDisconnectedAlert =
      new Alert("Climb Motor Disconnected", AlertType.kError);

  public Climb(ClimbIO io) {
    this.climbIO = io;
  }

  /**
   * Periodic function for the Climb subsystem. Updates the inputs and alerts the driver if the
   * climb motor is disconnected and the robot is in real mode.
   */
  @Override
  public void periodic() {
    climbIO.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);

    climbMotorDisconnectedAlert.set(!inputs.climbConnected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Creates an InstantCommand that sets the climb motor to the specified position in radians.
   *
   * @param position The target position for the climb motor in radians.
   * @return An InstantCommand that sets the climb motor to the specified position.
   */
  public Command setClimbPosition(double position) {
    return new InstantCommand(() -> climbIO.setClimbPosition(position), this);
  }

  /**
   * Creates an InstantCommand that sets the climb motor to the specified velocity in radians per
   * second.
   *
   * @param velocity The target velocity for the climb motor in radians per second.
   * @return An InstantCommand that sets the climb motor to the specified velocity.
   */
  public Command setClimbVelocity(double velocity) {
    return new InstantCommand(() -> climbIO.setClimbVelocity(velocity), this);
  }
}

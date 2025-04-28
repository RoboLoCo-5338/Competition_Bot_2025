package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  public final ArmIO io;
  public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  public double armPosition;

  private final Alert armDisconnectedAlert =
      new Alert("Arm motor disconnected", AlertType.kWarning);

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    //updates inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    //updates arm position
    armPosition = io.getArmPosition(inputs);
    //decides whether or not to set armDisconnectedAlert
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
    //runs setArmPosition and the .until() checks if the current armPosition is within the tolerance in order to end the command
    return new StartEndCommand(() -> io.setArmPosition(position), () -> io.setArmVelocity(0), this)
        .until(() -> Math.abs((inputs.armPosition - position)) < ArmConstants.POSITION_TOLERANCE);
  }

  /**
   * Sets the arm to the given velocity in degrees per second.
   *
   * <p>This is a non-blocking call and will not wait until the arm is at the requested velocity.
   *
   * @param velocity The velocity to set the arm to in degrees per second.
   * @return A command that sets the arm to the given velocity.
   */
  public Command setArmVelocity(DoubleSupplier velocity) {
    return new InstantCommand(() -> io.setArmVelocity(velocity.getAsDouble()), this);
  }

  public DoubleSupplier getArmPosition() {
    //gets arm position from the autologger
    SmartDashboard.putNumber("Getting arm position in Arm.java", armPosition);
    SmartDashboard.putNumber("Getting in arm.java 2", io.getArmPosition(inputs));
    return () -> io.getArmPosition(inputs);
  }
}

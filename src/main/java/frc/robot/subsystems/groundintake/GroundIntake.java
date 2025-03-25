package frc.robot.subsystems.groundintake;

// import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class GroundIntake extends SubsystemBase {

  public final GroundIntakeIO io;
  private final GroundIntakeIOInputsAutoLogged inputs = new GroundIntakeIOInputsAutoLogged();

  private final Alert armDisconnectedAlert =
      new Alert("Ground Intake arm motor disconnected", AlertType.kError);
  private final Alert intakeDisconnectedAlert =
      new Alert("Ground Intake intake motor disconnected", AlertType.kError);

  public GroundIntake(GroundIntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Ground Intake", inputs);

    armDisconnectedAlert.set(!inputs.armMotorConnected && Constants.currentMode != Mode.SIM);
    intakeDisconnectedAlert.set(!inputs.intakeMotorConnected && Constants.currentMode != Mode.SIM);
  }

  public Command setGroundIntakeVelocity(double velocity) {
    System.out.println("hello");
    return new InstantCommand(() -> io.setIntakeVelocity(velocity), this);
  }

  public Command setGroundArmPosition(double position) {
    return new InstantCommand(() -> io.setArmPosition(position), this);
  }

  public Command setGroundArmVelocity(DoubleSupplier velocity) {
    return new InstantCommand(() -> io.setArmVelocity(velocity.getAsDouble()), this);
  }

  public Command setIntakeSpeed(double speed) {
    return new InstantCommand(() -> io.setIntakeSpeed(-1));
  }
}

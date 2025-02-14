package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final Alert armDisconnectedAlert =
      new Alert("Arm motor disconnected", AlertType.kWarning);

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    armDisconnectedAlert.set(!inputs.armConnected && Constants.currentMode != Mode.SIM);
  }

  public Command setArmPosition(double position) {
    return new InstantCommand(() -> io.setArmPosition(position));
  }

  public Command setArmVelocity(double velocity) {
    return new InstantCommand(() -> io.setArmVelocity(velocity));
  }
}

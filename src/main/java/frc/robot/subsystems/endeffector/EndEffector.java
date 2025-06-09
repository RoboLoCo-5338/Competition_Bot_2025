package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SysIDSubsystem;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SysIDSubsystem {

  public final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  private final Alert endEffectorDisconnectedAlert =
      new Alert(" End Effector motor disconnected!!", AlertType.kError);

  private final SysIdRoutine sysIdRoutine;

  public EndEffector(EndEffectorIO io) {
    this.io = io;
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofBaseUnits(0.2, Volts.per(Second)),
                Voltage.ofBaseUnits(2, Volts),
                Second.of(30),
                (state) -> Logger.recordOutput("EndEffector/SysIdState", state.toString())),
            new Mechanism(io::endEffectorOpenLoop, null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("End Effector", inputs);

    endEffectorDisconnectedAlert.set(
        !inputs.endEffectorConnected && Constants.currentMode != Mode.SIM);
  }

  public Command setEndEffectorVelocity(double velocity) {
    return new InstantCommand(
        () -> {
          io.setEndEffectorVelocity(velocity);
        },
        this);
  }

  public Command setEndEffectorSpeed(double speed) {
    return new InstantCommand(() -> io.setEndEffectorSpeed(speed));
  }

  @Override
  public String getName() {
    return "End Effector";
  }
}

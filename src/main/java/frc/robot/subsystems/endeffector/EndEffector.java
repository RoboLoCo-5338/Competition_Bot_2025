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

public class EndEffector extends SubsystemBase implements SysIDSubsystem {
  //new input/output w/ motor
  public final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  //alert for when the end effector is disconnected
  private final Alert endEffectorDisconnectedAlert =
      new Alert(" End Effector motor disconnected!!", AlertType.kError);

  private final SysIdRoutine sysIdRoutine;

  /**
   * Constructor for the End Effector subsystem.
   * @param io EndEffectorIO implementation to use (e.g., EndEffectorIOTalonFX)
   */
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
            this)
        .withName("Set End Effector Velocity");
  }
  /**instant command to set end effector speed */
  public Command setEndEffectorSpeed(double speed) {
    return new InstantCommand(() -> io.setEndEffectorSpeed(speed))
        .withName("Set End Effector Speed");
  }

  /**
   * Returns the IO implementation used by this subsystem.
   * @return hardware IO class
   */
  public EndEffectorIO getIO() {
    return io;
  }

  /**
   * Runs the sysid quasistatic routine for end effector
   * @param direction
   * @return
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction).withName("SysId Quasistatic");
  }

  /**
   * Runs the sysid dynamic routine for end effector
   * @param direction
   * @return
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction).withName("SysId Dynamic");
  }

  /**
   * Returns the sysid routine for the end effector
   */
  @Override
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }

  @Override
  public String getName() {
    return "End Effector ";
  }
}

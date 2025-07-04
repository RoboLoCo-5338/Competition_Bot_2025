package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SysIDSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase implements SysIDSubsystem {

  public final ArmIO io;
  public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  public double armPosition;
  private final SysIdRoutine sysIdRoutine;

  private final Alert armDisconnectedAlert =
      new Alert("Arm motor disconnected", AlertType.kWarning);

  public Arm(ArmIO io) {
    this.io = io;
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new Mechanism(io::armOpenLoop, null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    armPosition = io.getArmPosition(inputs);

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
    return new StartEndCommand(() -> io.setArmPosition(position), () -> io.setArmVelocity(0), this)
        .until(() -> Math.abs((inputs.armPosition - position)) < ArmConstants.POSITION_TOLERANCE)
        .withName("Set Arm Position");
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
    return new InstantCommand(() -> io.setArmVelocity(velocity.getAsDouble()), this)
        .withName("Set Arm Velocity");
  }

  public DoubleSupplier getArmPosition() {
    SmartDashboard.putNumber("Getting arm position in Arm.java", armPosition);
    SmartDashboard.putNumber("Getting in arm.java 2", io.getArmPosition(inputs));
    return () -> io.getArmPosition(inputs);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction).withName("SysId Quasistatic");
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction).withName("SysId Dynamic");
  }

  @Override
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }

  @Override
  public String getName() {
    return "Arm ";
  }
}

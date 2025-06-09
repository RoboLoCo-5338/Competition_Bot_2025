package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.SysIDSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SysIDSubsystem<ArmIO, ArmIOInputsAutoLogged> {
  public double armPosition;
  private boolean armPositionRunning = false;
  public Arm(ArmIO io) {
    super(
        io,
        new ArmIOInputsAutoLogged(),
        new SysIdRoutine.Config(
            null, null, null, (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())));
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
    return new StartEndCommand(
            () -> {
              io.setArmPosition(position);
              armPositionRunning = true;
            },
            () -> {
              io.setArmVelocity(0);
              armPositionRunning = false;
            },
            this)
        .until(
            new Trigger(() -> Math.abs(inputs.velocity) < 0.001)
                .and(() -> armPositionRunning)
                .debounce(0.5)
                .onTrue(
                    new InstantCommand()) // Why the heck does this need to be here? The code breaks
                // if it's not there.
                .or(
                    () ->
                        Math.abs((inputs.position - position))
                            < ArmConstants.POSITION_TOLERANCE));
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

  public double getArmPosition() {
    SmartDashboard.putNumber("Getting arm position in Arm.java", armPosition);
    SmartDashboard.putNumber("Getting in arm.java 2", io.getArmPosition(inputs));
    return io.getArmPosition(inputs);
  }

  @Override
  public String getName() {
    return "Arm";
  }
}

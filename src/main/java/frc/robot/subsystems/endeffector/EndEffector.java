package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SysIDSubsystem;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SysIDSubsystem<EndEffectorIO, EndEffectorIOInputsAutoLogged> {

  public EndEffector(EndEffectorIO io) {
    super(
        io,
        new EndEffectorIOInputsAutoLogged(),
        new SysIdRoutine.Config(
            Velocity.ofBaseUnits(0.2, Volts.per(Second)),
            Voltage.ofBaseUnits(2, Volts),
            Second.of(30),
            (state) -> Logger.recordOutput("EndEffector/SysIdState", state.toString())));
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

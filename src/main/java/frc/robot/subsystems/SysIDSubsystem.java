package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SysIDIO.SysIDIOInputs;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public abstract class SysIDSubsystem<T extends SysIDIO<I>, I extends SysIDIOInputs & LoggableInputs>
    extends SingleIOSubsystem<T, I> {
  private final SysIdRoutine sysIdRoutine;

  public SysIDSubsystem(T io, I inputs, SysIdRoutine.Config config) {
    super(io, inputs);
    sysIdRoutine = new SysIdRoutine(config, new SysIdRoutine.Mechanism(io::openLoop, null, this));
  }

  public void addRoutinesToChooser(LoggedDashboardChooser<Command> autoChooser) {
    autoChooser.addOption(
        getName() + " SysId Quasistatic Forward",
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        getName() + " SysId Quasistatic Backward",
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        getName() + " SysId Dynamic Forward",
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        getName() + " SysId Dynamic Backward",
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}

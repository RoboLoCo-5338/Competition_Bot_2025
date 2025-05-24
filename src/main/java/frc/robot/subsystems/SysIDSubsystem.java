package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public interface SysIDSubsystem {
  public SysIdRoutine getSysIdRoutine();

  public String getName();

  public default void addRoutinesToChooser(LoggedDashboardChooser<Command> autoChooser) {
    SysIdRoutine sysIdRoutine = getSysIdRoutine();
    autoChooser.addOption(
        getName() + "SysId Quasistatic Forward",
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        getName() + "SysId Quasistatic Backward",
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        getName() + "SysId Dynamic Forward", sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        getName() + "SysId Dynamic Backward",
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "All Sysid Routines(Recommended)",
        new SequentialCommandGroup(
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            new WaitCommand(5),
            sysIdRoutine.quasistatic(Direction.kReverse),
            new WaitCommand(5),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
            new WaitCommand(5),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)));
  }
}

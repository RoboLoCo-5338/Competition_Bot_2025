package frc.robot.subsystems.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmPresetConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPresetConstants;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EndEffectorSimConstants;
import frc.robot.util.Level;
import frc.robot.util.PoseUtils;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class ReefAlgaeSimHandler extends SubsystemBase {
  private class ReefSegment {
    public boolean hasAlgae;
    public Level algaeLevel;
    public Pose2d intakePose;
    public Pose3d algaePose;

    public ReefSegment(int tagIndex, Alliance alliance) {
      hasAlgae = true;
      if (alliance == Alliance.Red) algaeLevel = (tagIndex % 2 == 0) ? Level.L3 : Level.L2;
      else algaeLevel = (tagIndex % 2 == 0) ? Level.L2 : Level.L3;
      intakePose = PoseUtils.tagRotate(DriveConstants.reefCenter, tagIndex, alliance);
      algaePose =
          new Pose3d(PoseUtils.tagRotate(EndEffectorSimConstants.ALGAE_REEF, tagIndex, alliance))
              .plus(
                  new Transform3d(0, 0, (algaeLevel == Level.L2) ? 0.93 : 1.35, new Rotation3d()));
    }
  }

  public ReefSegment[] reefSegments;

  public ReefAlgaeSimHandler(Alliance alliance) {
    reefSegments = new ReefSegment[6];
    int sT = (alliance == Alliance.Red) ? 6 : 17;
    for (int i = sT; i < sT + 6; i++) {
      reefSegments[i - sT] = new ReefSegment(i, alliance);
    }
  }

  public boolean canIntakeAlgae(Pose2d pose, Arm arm, Elevator elevator) {
    for (ReefSegment segment : reefSegments) {
      if (PoseUtils.arePosesSimilar(
          pose,
          segment.intakePose,
          DriveConstants.AUTO_ALIGN_X_TOLERANCE,
          DriveConstants.AUTO_ALIGN_ANGULAR_TOLERANCE)) {
        boolean o =
            segment.hasAlgae
                && Math.abs(arm.getArmPosition() - ArmPresetConstants.ARM_ALGAE)
                    < ArmConstants.POSITION_TOLERANCE
                && Math.abs(
                        elevator.getElevatorPosition()
                            - ((segment.algaeLevel == Level.L2)
                                ? ElevatorPresetConstants.ELEVATOR_L2_ALGAE
                                : ElevatorPresetConstants.ELEVATOR_L3_ALGAE))
                    < ElevatorConstants.POSITION_TOLERANCE;
        if (o) segment.hasAlgae = false; // consume algae
        return o;
      }
    }
    return false;
  }

  @Override
  public void periodic() {
    ArrayList<Pose3d> algaeRenderPoses = new ArrayList<>();
    for (ReefSegment segment : reefSegments) {
      if (segment.hasAlgae) {
        algaeRenderPoses.add(segment.algaePose);
      }
    }
    Pose3d[] o = new Pose3d[algaeRenderPoses.size()];
    Logger.recordOutput("FieldSimulation/AlgaeReef", algaeRenderPoses.toArray(o));
  }
}

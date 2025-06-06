package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmPresetConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPresetConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.Level;
import frc.robot.util.PoseUtils;

public class ReefAlgaeSimHandler {
  private class ReefSegment {
    public boolean hasAlgae;
    public Level algaeLevel;
    public Pose2d intakePose;

    public ReefSegment(int tagIndex, Alliance alliance) {
      boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
      hasAlgae = true;
      if (alliance == Alliance.Red) algaeLevel = (tagIndex % 2 == 0) ? Level.L2 : Level.L3;
      else algaeLevel = (tagIndex % 2 == 0) ? Level.L3 : Level.L2;
      Rotation2d rot =
          VisionConstants.aprilTagLayout.getTagPose(tagIndex).get().getRotation().toRotation2d();
      if (isFlipped) rot = rot.plus(new Rotation2d(Math.PI));
      intakePose =
          PoseUtils.allianceFlip(
              DriveConstants.reefCenter.rotateAround(new Translation2d(4.5, 4.03), rot));
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
          DriveConstants.AUTO_ALIGN_ANGULAR_TOLERANCE))
        return segment.hasAlgae
            && Math.abs(arm.getArmPosition() - ArmPresetConstants.ARM_ALGAE)
                < ArmConstants.POSITION_TOLERANCE
            && Math.abs(
                    elevator.getElevatorPosition()
                        - ((segment.algaeLevel == Level.L2)
                            ? ElevatorPresetConstants.ELEVATOR_L2_ALGAE
                            : ElevatorPresetConstants.ELEVATOR_L3_ALGAE))
                < ElevatorConstants.POSITION_TOLERANCE;
    }
    return false;
  }
}

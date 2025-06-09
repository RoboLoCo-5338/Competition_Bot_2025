package frc.robot.util;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionConstants;

public class PoseUtils {
  public static Pose2d allianceFlip(Pose2d pose, Alliance alliance) {
    return (alliance == Alliance.Red) ? FlippingUtil.flipFieldPose(pose) : pose;
  }

  public static Pose2d allianceFlip(Pose2d pose) {
    return allianceFlip(pose, DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  public static double distanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
    return pose1.minus(pose2).getTranslation().getNorm();
  }

  public static boolean arePosesSimilar(
      Pose2d pose1, Pose2d pose2, double linearTolerance, double angularTolerance) {
    return distanceBetweenPoses(pose1, pose2) < linearTolerance
        && MathUtil.angleModulus(pose1.getRotation().getRadians())
                - MathUtil.angleModulus(pose2.getRotation().getRadians())
            < angularTolerance;
  }

  public static Pose2d tagRotate(Pose2d pose, int tag, Alliance alliance) {
    boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    Rotation2d rot =
        VisionConstants.aprilTagLayout.getTagPose(tag).get().getRotation().toRotation2d();
    if (isFlipped) rot = rot.plus(new Rotation2d(Math.PI));
    return allianceFlip(pose.rotateAround(new Translation2d(4.5, 4.03), rot));
  }
}

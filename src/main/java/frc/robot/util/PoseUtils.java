package frc.robot.util;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PoseUtils {
    public static Pose2d allianceFlip(Pose2d pose, Alliance alliance){
        return (alliance==Alliance.Red)? FlippingUtil.flipFieldPose(pose) : pose;
    }
    public static Pose2d allianceFlip(Pose2d pose){
        return allianceFlip(pose, DriverStation.getAlliance().orElse(Alliance.Blue));
    }
    public static double distanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
        return pose1.minus(pose2).getTranslation().getNorm();
    }
    public static boolean arePosesSimilar(Pose2d pose1, Pose2d pose2, double linearTolerance, double angularTolerance) {
        return distanceBetweenPoses(pose1, pose2) < linearTolerance && MathUtil.angleModulus(pose1.getRotation().getRadians()) - MathUtil.angleModulus(pose2.getRotation().getRadians()) < angularTolerance;
    }
}

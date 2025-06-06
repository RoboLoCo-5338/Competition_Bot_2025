package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.Level;
import frc.robot.util.PoseUtils;

public class ReefAlgaeSimHandler {
  public class ReefSegment {
    public boolean hasAlgae;
    public Level algaeLevel;
    public Pose2d intakePose;
    public ReefSegment(int tagIndex, Alliance alliance){
        boolean isFlipped=DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        hasAlgae=true;
        if(alliance==Alliance.Red) algaeLevel = (tagIndex%2==0)? Level.L2 : Level.L3;
        else algaeLevel = (tagIndex%2==0)? Level.L3 : Level.L2;
        Rotation2d rot =
          VisionConstants.aprilTagLayout.getTagPose(tagIndex).get().getRotation().toRotation2d();
        if (isFlipped) rot = rot.plus(new Rotation2d(Math.PI));
        intakePose = PoseUtils.allianceFlip(DriveConstants.reefCenter.rotateAround(new Translation2d(4.5, 4.03), rot)); //Don't ask me why I'm using a func from DriveCommands
    }
  }
}

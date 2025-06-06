package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class EndEffectorConstants {
  public static final int EFFECTORID = 43;
  public static final int EFFECTOR_CURRENT_LIMIT = 200;
  public static final double EFFECTOR_KP = 0.1;
  public static final double EFFECTOR_KI = 0.0;
  public static final double EFFECTOR_KD = 0.0;
  public static final double EFFECTOR_KS = 0.0;
  public static final double EFFECTOR_KV = 0.0;
  public static final double EFFECTOR_KFF = 0.0;
  public static final double EFFECTOR_KG = 0.0;
  public static final int LASERCAN_1ID = 51;
  public static final int LASERCAN_2ID = 52;
  public static final double GEARING = 1;

  public static final class EndEffectorSimConstants {
    public static final double MOI = 0.0002048478;
    public static final double FRONT_ROLLER_ORIGIN_X = 0.318926;
    public static final double FRONT_ROLLER_ORIGIN_Y = -0.00108115;
    public static final double FRONT_ROLLER_ORIGIN_Z = 0.370936;
    public static final double BACK_ROLLER_ORIGIN_X = 0.197628;
    public static final double BACK_ROLLER_ORIGIN_Y = -0.00526875;
    public static final double BACK_ROLLER_ORIGIN_Z = 0.440842;

    public static final Pose2d ALGAE_REEF = new Pose2d(3.788, 4.031, new Rotation2d());
  }
}

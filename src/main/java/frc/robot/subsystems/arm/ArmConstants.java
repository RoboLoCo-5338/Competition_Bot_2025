package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public final class ArmConstants {
  public static final int ARM_MOTOR_ID = 44;
  public static final double ARM_MOTOR_POSITION_KP = 4.5;
  public static final double ARM_MOTOR_POSITION_KI = 0.1;
  public static final double ARM_MOTOR_POSITION_KD = 0.04;
  public static final double ARM_MOTOR_VELOCITY_KP = 0.005;
  public static final double ARM_MOTOR_VELOCITY_KI = 0.1;
  public static final double ARM_MOTOR_VELOCITY_KD = 0.0;
  public static final double ARM_MOTOR_KG = 0.4375;
  public static final double ARM_MOTOR_KV = 0.0;
  public static final double ARM_MOTOR_KS = 0.0;
  public static final int ARM_MOTOR_CURRENT_LIMIT = 600;
  public static final double POSITION_TOLERANCE = 0.02; // native units
  public static final double ENCODER_GEARING = 1.0 / 1.125;

  public static final class ArmSimConstants {
    public static final double LENGTH = Units.inchesToMeters(22.9);
    public static final double MOI =
        SingleJointedArmSim.estimateMOI(ArmSimConstants.LENGTH, Units.lbsToKilograms(5.3754589));
    public static final double GEARING = 50;
    public static final double MIN_ANGLE = Units.degreesToRadians(-103);
    public static final double MAX_ANGLE = Units.degreesToRadians(90);
    public static final double STARTING_ANGLE = Units.degreesToRadians(0);
    public static final double SIM_OFFSET = 0.672; // rotations

    public static final double ORIGIN_X = 0.2413; // meters
    public static final double ORIGIN_Y = -0.003175; // meters
    public static final double ORIGIN_Z = 0.79375; // meters
  }
}

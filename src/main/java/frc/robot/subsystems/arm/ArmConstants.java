package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public final class ArmConstants {
  public static final int ARM_MOTOR_ID = 44;
  public static final double ARM_MOTOR_POSITION_KP = 2.5; // 3.5
  public static final double ARM_MOTOR_POSITION_KI = 0.00; // 0.01
  public static final double ARM_MOTOR_POSITION_KD = 0.25;
  public static final double ARM_MOTOR_VELOCITY_KP = 0.005; // 3.5
  public static final double ARM_MOTOR_VELOCITY_KI = 0.00; // 0.01
  public static final double ARM_MOTOR_VELOCITY_KD = 0.0005;
  public static final double ARM_MOTOR_KG = 0.25;
  public static final double ARM_MOTOR_KV = 0.00;
  public static final double ARM_MOTOR_KS = 0.0; // 0.1
  public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
  public static final double POSITION_TOLERANCE = 0.01; // %
  public static final double GEARING = 1.0 / 1.125;

  public static final class ArmSimConstants {
    public static final double LENGTH = Units.inchesToMeters(22.9);
    public static final double MOI =
        SingleJointedArmSim.estimateMOI(ArmSimConstants.LENGTH, Units.lbsToKilograms(5.3754589));
    public static final double MIN_ANGLE = Units.degreesToRadians(-90);
    public static final double MAX_ANGLE = Units.degreesToRadians(103);
    public static final double STARTING_ANGLE = Units.degreesToRadians(-90);
  }
}

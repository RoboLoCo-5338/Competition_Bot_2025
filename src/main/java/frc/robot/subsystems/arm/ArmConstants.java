package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public final class ArmConstants {
  public static final int ARM_MOTOR_ID = 44;
  public static final double ARM_MOTOR_POSITION_KP = 5;
  public static final double ARM_MOTOR_POSITION_KI = 0.00;
  public static final double ARM_MOTOR_POSITION_KD = 0.0;
  public static final double ARM_MOTOR_VELOCITY_KP = 0.001;
  public static final double ARM_MOTOR_VELOCITY_KI = 0.00;
  public static final double ARM_MOTOR_VELOCITY_KD = 0.0;
  public static final double ARM_MOTOR_KG = 0.27;
  public static final double ARM_MOTOR_KV = 0.00;
  public static final double ARM_MOTOR_KS = 0.0;
  public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
  public static final double POSITION_TOLERANCE = 0.005; // native units
  public static final double SOFT_LIMIT = -0.283; // TODO: update based on new end effector arm

  public static final class ArmSimConstants {
    public static final double LENGTH = Units.inchesToMeters(21.8);
    public static final double MOI =
        SingleJointedArmSim.estimateMOI(ArmSimConstants.LENGTH, Units.lbsToKilograms(3.9));
    public static final double GEARING = 50;
    public static final double MIN_ANGLE = Units.degreesToRadians(-103);
    public static final double MAX_ANGLE = Units.degreesToRadians(90);
    public static final double STARTING_ANGLE = 0;

    public static final double ORIGIN_X = 0.2413; // meters
    public static final double ORIGIN_Y = -0.003175; // meters
    public static final double ORIGIN_Z = 0.79375; // meters
  }

  public static final class ArmPresetConstants {
    public static final double ARM_L2_L3 =
        -0.183; // TODO: update these values for new end effector arm
    public static final double ARM_L4 = 0.075;
    public static final double ARM_NET = 0.25;
    public static final double ARM_ALGAE = -0.249; // TODO: update
    public static final double ARM_STOW_INITIAL = -0.264 + 0.09;
    public static final double ARM_STOW_FINAL = -0.25;
  }
}

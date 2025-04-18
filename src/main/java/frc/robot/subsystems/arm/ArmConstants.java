package frc.robot.subsystems.arm;

import com.roboloco.tune.IsTunableConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

@IsTunableConstants
public class ArmConstants {
  public static final int ARM_MOTOR_ID = 44;
  public static double ARM_MOTOR_POSITION_KP = 2.5;
  public static double ARM_MOTOR_POSITION_KI = 0.00;
  public static double ARM_MOTOR_POSITION_KD = 0.25;
  public static double ARM_MOTOR_VELOCITY_KP = 0.005;
  public static double ARM_MOTOR_VELOCITY_KI = 0.00;
  public static double ARM_MOTOR_VELOCITY_KD = 0.0005;
  public static double ARM_MOTOR_KG = 0.25;
  public static double ARM_MOTOR_KV = 0.00;
  public static double ARM_MOTOR_KS = 0.0;
  public static int ARM_MOTOR_CURRENT_LIMIT = 60;
  public static double POSITION_TOLERANCE = 0.01; // native units
  public static double SOFT_LIMIT = 0.43; // TODO: update based on new end effector arm

  public static final class ArmSimConstants {
    public static final double GEARING = 1.0 / 1.125;
    public static final double LENGTH = Units.inchesToMeters(22.9);
    public static final double MOI =
        SingleJointedArmSim.estimateMOI(ArmSimConstants.LENGTH, Units.lbsToKilograms(5.3754589));
    public static final double MIN_ANGLE = Units.degreesToRadians(-90);
    public static final double MAX_ANGLE = Units.degreesToRadians(103);
    public static final double STARTING_ANGLE = Units.degreesToRadians(-90);
  }

  @IsTunableConstants
  public static class ArmPresetConstants {
    public static double ARM_L2_L3 = 0.54; // TODO: update these values for new end effector arm
    public static double ARM_L4 = 0.78;
    public static double ARM_NET = 0.762;
    public static double ARM_L3_ALGAE = 0.0;
  }
}

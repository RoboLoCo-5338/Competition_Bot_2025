package frc.robot.subsystems.climb;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public final class ClimbConstants {
  public static final int CLIMB_MOTOR_ID = 46;
  public static final int CLIMB_MOTOR_CURRENT_LIMIT = 120;
  public static final double CLIMB_kP = 0.5;
  public static final double CLIMB_kI = 0.2;
  public static final double CLIMB_kD = 0.0;
  public static final double CLIMB_kS = 0.0;
  // Sim Constants
public static final class ClimbSimConstants {
public static final double GEARING = 100; // Update this pls don't worry bro I did
public static final double BASE_HEIGHT = Units.inchesToMeters(6.25);
public static final double ARM_LENGTH = Units.inchesToMeters(12.1);
public static final double MOI =
        SingleJointedArmSim.estimateMOI(ARM_LENGTH, Units.lbsToKilograms(2.2));
public static final double MIN_ANGLE = Units.degreesToRadians(-148.47);
public static final double MAX_ANGLE = Units.degreesToRadians(-90);
public static final double STARTING_ANGLE = Units.degreesToRadians(-148.47);
}
}

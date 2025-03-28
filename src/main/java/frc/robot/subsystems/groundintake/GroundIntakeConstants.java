package frc.robot.subsystems.groundintake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public final class GroundIntakeConstants {
  public static final class ArmConstants {
    public static final int ARM_CANID = 40; // CHANGED FOR TESTING
    public static final int ARM_CURRENT_LIMIT = 60;
    public static final double ARM_KP = 0.01;
    public static final double ARM_KI = 0;
    public static final double ARM_KD = 0;
    public static final double ARM_KFF = 0;
    public static final double ARM_KS = 0;
    public static final double ARM_KV = 0;
    // Sim stuff
    public static final class ArmSimConstants{
      public static final double GEARING = 1;
      public static final double LENGTH = Units.inchesToMeters(18.5);
      public static final double MOI =
          SingleJointedArmSim.estimateMOI(
              ArmSimConstants.LENGTH, Units.lbsToKilograms(4.9));
      public static final double MIN_ANGLE = Units.degreesToRadians(-106);
      public static final double MAX_ANGLE = Units.degreesToRadians(0);
      public static final double STARTING_ANGLE = Units.degreesToRadians(-106);
      public static final double ARM_BASE_HEIGHT = Units.inchesToMeters(6.25);
    }
  }

  public static final class IntakeConstants {
    public static final int INTAKE_CANID = 41; // CHANGED FOR TESTING
    public static final int INTAKE_CURRENT_LIMIT = 80;
    public static final double INTAKE_KP = 0.00045;
    public static final double INTAKE_KI = 0;
    public static final double INTAKE_KD = 0;
    public static final double INTAKE_KFF = 0;
    public static final double INTAKE_KS = 0;
    public static final double INTAKE_KV = 0;
    // Sim stuff
    public static final class IntakeSimConstants{
      public static final double MOI = 0.0002341117;
      public static final double GEARING = 1;
    }
  }
}

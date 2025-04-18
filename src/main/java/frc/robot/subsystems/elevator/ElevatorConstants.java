package frc.robot.subsystems.elevator;

import com.roboloco.tune.IsTunableConstants;
import edu.wpi.first.math.util.Units;

@IsTunableConstants
public class ElevatorConstants {
  public static final int ELEVATOR_MOTOR_ID1 = 42;
  public static final int ELEVATOR_MOTOR_ID2 = 45;
  public static int ELEVATOR_MOTOR_CURRENT_LIMIT = 40;
  public static final double LASERCAN_TO_ELEVATOR_POSITION = 1.0;
  // public static final double ELEVATOR_EPSILON = 1e-2;
  // public static final double ELEVATOR_kP_LASERCAN = 0.0;
  // public static final double ELEVATOR_kI_LASERCAN = 0.0;
  // public static final double ELEVATOR_kD_LASERCAN = 0.0;
  public static double POSITION_TOLERANCE = 0.1; // native units

  @IsTunableConstants
  public static class ElevatorPositionConstants {
    public static double ELEVATOR_FEEDFORWARD = 0.0;
    public static double ELEVATOR_MOTOR_kP = 1.2; // 0.3
    public static double ELEVATOR_MOTOR_kI = 0.03;
    public static double ELEVATOR_MOTOR_kD = 0.013;
    public static double ELEVATOR_MOTOR_kG = 0.5; // 0.5
    public static double ELEVATOR_MOTOR_kV = 0.0;
  }

  @IsTunableConstants
  public static class ElevatorVelocityConstants {
    public static double ELEVATOR_FEEDFORWARD = 0.0;
    public static double ELEVATOR_MOTOR_kP = 0.1; // 0.15
    public static double ELEVATOR_MOTOR_kI = 0.0; // 0.03
    public static double ELEVATOR_MOTOR_kD = 0.00;
    public static double ELEVATOR_MOTOR_kG = 0.5; // 0.5
    public static double ELEVATOR_MOTOR_kV = 0.0;
  }

  @IsTunableConstants
  public static class ElevatorStowPresetConstants {
    public static double ELEVATOR_FEEDFORWARD = 0.0;
    public static double ELEVATOR_MOTOR_kP = 0.28; // 0.3
    public static double ELEVATOR_MOTOR_kI = 0.02;
    public static double ELEVATOR_MOTOR_kD = 0.00;
    public static double ELEVATOR_MOTOR_kG = 0.5; // 0.5
    public static double ELEVATOR_MOTOR_kV = 0.0;
  }

  public static final class ElevatorSimConstants {
    public static final double GEARING = 4;
    public static final double CARRIAGE_MASS = Units.lbsToKilograms(17.1910833);
    public static final double DRUM_RADIUS = Units.inchesToMeters(1.44);
    public static final double MIN_HEIGHT = Units.inchesToMeters(1);
    public static final double MAX_HEIGHT = Units.inchesToMeters(53);
    public static final double STARTING_HEIGHT = Units.inchesToMeters(1);
    public static final double METERS_PER_ROTATION =
        2 * ElevatorSimConstants.DRUM_RADIUS * Math.PI / ElevatorSimConstants.GEARING;
  }

  @IsTunableConstants
  public static class ElevatorPresetConstants {
    public static double ELEVATOR_L2 = 7.9;
    public static double ELEVATOR_L3 = 14.708;
    public static double ELEVATOR_L4 = 19.40;
    public static double ELEVATOR_NET = 16.486; // TODO: update
    public static double ELEVATOR_L3_ALGAE = 0.0;
    public static double ELEVATOR_STOW = 0.05;
  }
}

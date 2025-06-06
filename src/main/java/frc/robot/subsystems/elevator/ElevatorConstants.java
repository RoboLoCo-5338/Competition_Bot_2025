package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public final class ElevatorConstants {
  public static final int ELEVATOR_MOTOR_ID1 = 42;
  public static final int ELEVATOR_MOTOR_ID2 = 45;
  public static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 400;
  public static final double LASERCAN_TO_ELEVATOR_POSITION = 1.0;
  public static final double ELEVATOR_EPSILON = 1e-2;
  public static final double ELEVATOR_kP_LASERCAN = 0.0;
  public static final double ELEVATOR_kI_LASERCAN = 0.0;
  public static final double ELEVATOR_kD_LASERCAN = 0.0;
  public static final double POSITION_TOLERANCE = 0.001; // native units
  public static final double GEARING = 4;
  public static final double METERS_PER_ROTATION =
      2 * ElevatorSimConstants.DRUM_RADIUS * Math.PI / ElevatorConstants.GEARING;

  public static final class ElevatorPositionConstants {
    public static final double ELEVATOR_FEEDFORWARD = 0.0;
    public static final double ELEVATOR_MOTOR_kP = 1.5; // 0.3
    public static final double ELEVATOR_MOTOR_kI = 0.03;
    public static final double ELEVATOR_MOTOR_kD = 0.15;
    public static final double ELEVATOR_MOTOR_kG = 0.5; // 0.5
    public static final double ELEVATOR_MOTOR_kV = 0.0;
  }

  public static final class ElevatorVelocityConstants {
    public static final double ELEVATOR_FEEDFORWARD = 0.0;
    public static final double ELEVATOR_MOTOR_kP = 0.2;
    public static final double ELEVATOR_MOTOR_kI = 0.0;
    public static final double ELEVATOR_MOTOR_kD = 0.0;
    public static final double ELEVATOR_MOTOR_kG = 0.510152;
    public static final double ELEVATOR_MOTOR_kV = 0.0;
  }

  public static class ElevatorStowPresetConstants {
    public static final double ELEVATOR_FEEDFORWARD = 0.0;
    public static final double ELEVATOR_MOTOR_kP = 0.28;
    public static final double ELEVATOR_MOTOR_kI = 0.02;
    public static final double ELEVATOR_MOTOR_kD = 0.00;
    public static final double ELEVATOR_MOTOR_kG = 0.5;
    public static final double ELEVATOR_MOTOR_kV = 0.0;
  }

  public static final class ElevatorSimConstants {
    public static final double CARRIAGE_MASS =
        Units.lbsToKilograms(19.119058) / 2; // Two motors sharing the weight
    public static final double DRUM_RADIUS = Units.inchesToMeters(1.44);
    public static final double MIN_HEIGHT = Units.inchesToMeters(0);
    public static final double MAX_HEIGHT = Units.inchesToMeters(56);
    public static final double STARTING_HEIGHT = Units.inchesToMeters(1);
    public static final double STAGE_1_ORIGIN_X = 0.1651; // meters
    public static final double STAGE_1_ORIGIN_Y = 0.0; // meters
    public static final double STAGE_1_ORIGIN_Z = 0.10795; // meters
    public static final double STAGE_2_ORIGIN_X = 0.1651; // meters
    public static final double STAGE_2_ORIGIN_Y = 0.0; // meters
    public static final double STAGE_2_ORIGIN_Z = 0.13335; // meters
  }

  public static final class ElevatorPresetConstants {
    public static final double ELEVATOR_L2 = 9.486 * ElevatorConstants.METERS_PER_ROTATION;
    public static final double ELEVATOR_L3 = 16.6 * ElevatorConstants.METERS_PER_ROTATION;
    public static final double ELEVATOR_L4 = 19.40 * ElevatorConstants.METERS_PER_ROTATION;
    public static final double ELEVATOR_NET =
        16.486 * ElevatorConstants.METERS_PER_ROTATION; // TODO: update
    public static final double ELEVATOR_L2_ALGAE =
        9.486 * ElevatorConstants.METERS_PER_ROTATION; // TODO update
    public static final double ELEVATOR_L3_ALGAE = 1.302;
    public static final double ELEVATOR_STOW = 0.05 * ElevatorConstants.METERS_PER_ROTATION;
  }
}

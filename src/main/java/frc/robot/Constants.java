// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final double ROBOT_LENGTH = Units.inchesToMeters(33.250000);
  public static final double FLOOR_TO_MECHANISM = Units.inchesToMeters(8);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class PresetConstants {
    public static final double elevatorl2 = 10; // This was 17 for testing PID
    public static final double elevatorl3 = 16.95;
    public static final double elevatorl4 = 19.35;
    public static final double elevatorNet = 16.481;
    public static final double elevatorl3Algae = 0.0;

    public static final double arml2 = 0.53;
    public static final double arml3 = 0.53;
    public static final double arml4 = 0.800;
    public static final double armNet = 0.3;
    public static final double arml3Algae = 0.0;

    public static final double elevatorIntake = 0.0;
    public static final double armIntake = 0.0;
  }

  public static final class ArmConstants {
    public static final int ARM_MOTOR_ID = 44;
    public static final double ARM_MOTOR_KP = 5; // 3.5
    public static final double ARM_MOTOR_KI = 0.00; // 0.01
    public static final double ARM_MOTOR_KD = 0.0;
    public static final double ARM_MOTOR_KG = 0.0;
    public static final double ARM_MOTOR_KV = 0.00;
    public static final double ARM_MOTOR_KFF = 0.0;
    public static final double ARM_MOTOR_KS = 0.0; // 0.1
    public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
    // Sim constants
    public static final double GEARING = 1.0 / 1.125;
    public static final double LENGTH = Units.inchesToMeters(22.9);
    public static final double MOI =
        SingleJointedArmSim.estimateMOI(ArmConstants.LENGTH, Units.lbsToKilograms(5.3754589));
    public static final double MIN_ANGLE = Units.degreesToRadians(-90);
    public static final double MAX_ANGLE = Units.degreesToRadians(103);
    public static final double STARTING_ANGLE = Units.degreesToRadians(-90);
  }

  public static final class ClimbConstants {
    public static final int CLIMB_MOTOR_ID = 46;
    public static final int CLIMB_MOTOR_CURRENT_LIMIT = 120;
    public static final double CLIMB_kP = 0.5;
    public static final double CLIMB_kI = 0.2;
    public static final double CLIMB_kD = 0.0;
    public static final double CLIMB_kS = 0.0;
    // Sim Constants
    public static final double GEARING = 100; // Update this pls don't worry bro I did
    public static final double BASE_HEIGHT = Units.inchesToMeters(6.25);
    public static final double ARM_LENGTH = Units.inchesToMeters(12.1);
    public static final double MOI =
        SingleJointedArmSim.estimateMOI(ARM_LENGTH, Units.lbsToKilograms(2.2));
    public static final double MIN_ANGLE = Units.degreesToRadians(-148.47);
    public static final double MAX_ANGLE = Units.degreesToRadians(-90);
    public static final double STARTING_ANGLE = Units.degreesToRadians(-148.47);
    // test commits
  }

  public static final class ElevatorConstants {
    // TODO change non sim constants
    public static final int ELEVATOR_MOTOR_ID1 = 42;
    public static final int ELEVATOR_MOTOR_ID2 = 45;
    public static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 40;
    public static final int LASERCAN_ID = 50;

    public static final class ElevatorPositionConstants {
      public static final double ELEVATOR_FEEDFORWARD = 0.0;
      public static final double ELEVATOR_MOTOR_kP = 0.28; // 0.3
      public static final double ELEVATOR_MOTOR_kI = 0.02;
      public static final double ELEVATOR_MOTOR_kD = 0.00;
      public static final double ELEVATOR_MOTOR_kG = 0.5; // 0.5
      public static final double ELEVATOR_MOTOR_kV = 0.0;
    }

    public static final class ElevatorVelocityConstants {
      public static final double ELEVATOR_FEEDFORWARD = 0.0;
      public static final double ELEVATOR_MOTOR_kP = 0.15; // 0.15
      public static final double ELEVATOR_MOTOR_kI = 0.03; // 0.03
      public static final double ELEVATOR_MOTOR_kD = 0.00;
      public static final double ELEVATOR_MOTOR_kG = 0.6; // 0.5
      public static final double ELEVATOR_MOTOR_kV = 0.0;
    }

    public static final double LASERCAN_TO_ELEVATOR_POSITION = 1.0;
    public static final double ELEVATOR_EPSILON = 1e-2;
    public static final double ELEVATOR_kP_LASERCAN = 0.0;
    public static final double ELEVATOR_kI_LASERCAN = 0.0;
    public static final double ELEVATOR_kD_LASERCAN = 0.0;
    // Sim constants
    public static final double GEARING = 4;
    public static final double CARRIAGE_MASS = Units.lbsToKilograms(17.1910833);
    public static final double DRUM_RADIUS = Units.inchesToMeters(1.44);
    public static final double MIN_HEIGHT = Units.inchesToMeters(1);
    public static final double MAX_HEIGHT = Units.inchesToMeters(53);
    public static final double STARTING_HEIGHT = Units.inchesToMeters(1);
    public static final double METERS_PER_ROTATION =
        2 * ElevatorConstants.DRUM_RADIUS * Math.PI / ElevatorConstants.GEARING;
  }

  public static final class EndEffectorConstants {
    public static final int EFFECTORID = 43;
    public static final int EFFECTOR_CURRENT_LIMIT = 20;
    public static final double EFFECTOR_KP = 0.1;
    public static final double EFFECTOR_KI = 0.0;
    public static final double EFFECTOR_KD = 0.0;
    public static final double EFFECTOR_KS = 0.0;
    public static final double EFFECTOR_KV = 0.0;
    public static final double EFFECTOR_KFF = 0.0;
    public static final double EFFECTOR_KG = 0.05;
    public static final int LASERCAN_1ID = -1;
    public static final int LASERCAN_2ID = -1;
    // Sim Constants
    public static final double MOI = 0.0002048478;
    public static final double GEARING = 1;
  }

  public static final class GroundIntakeConstants {
    public static final class ArmConstants {
      // TODO change this
      public static final int ARM_CANID = 40; // CHANGED FOR TESTING
      public static final int ARM_CURRENT_LIMIT = 60;
      public static final double ARM_KP = 0.01;
      public static final double ARM_KI = 0;
      public static final double ARM_KD = 0;
      public static final double ARM_KFF = 0;
      public static final double ARM_KS = 0;
      public static final double ARM_KV = 0;
      // Sim stuff
      public static final double GEARING = 1;
      public static final double LENGTH = Units.inchesToMeters(18.5);
      public static final double MOI =
          SingleJointedArmSim.estimateMOI(
              GroundIntakeConstants.ArmConstants.LENGTH, Units.lbsToKilograms(4.9));
      public static final double MIN_ANGLE = Units.degreesToRadians(-106);
      public static final double MAX_ANGLE = Units.degreesToRadians(0);
      public static final double STARTING_ANGLE = Units.degreesToRadians(-106);
      public static final double ARM_BASE_HEIGHT = Units.inchesToMeters(6.25);
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
      public static final double MOI = 0.0002341117;
      public static final double GEARING = 1;
    }
  }

  public static final class LEDConstants {
    // TODO change this
    public static final int LED_PWM_PORT = 0;
    public static final int LED_LENGTH = 123;
    public static final int BARGE_RANGE = 1;
    public static final Distance LED_SPACING = Meters.of(1.0 / 50.0);
  }

  public class VisionConstants {
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "camera_0";
    public static String camera1Name = "camera_1";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(
            Units.inchesToMeters(11.12),
            -Units.inchesToMeters(9.77),
            Units.inchesToMeters(5.98),
            new Rotation3d(0.0, -Units.degreesToRadians(10), Units.degreesToRadians(22)));
    public static Transform3d robotToCamera1 =
        new Transform3d(
            Units.inchesToMeters(1.7),
            Units.inchesToMeters(.24),
            Units.inchesToMeters(36.7),
            new Rotation3d(0.0, -0.0, Math.PI));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          2.0 // Camera 1
        };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }
}

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
import edu.wpi.first.wpilibj.Preferences;
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

  public static final class DriveConstants {
    public static double pathPlannerDriveP = Preferences.getDouble("pathPlannerDriveP", 2.0);
    public static double pathPlannerDriveI = Preferences.getDouble("pathPlannerDriveI", 0.2);
    public static double pathPlannerDriveD = Preferences.getDouble("pathPlannerDriveD", 0.0);
    public static double pathPlannerTurnP = Preferences.getDouble("pathPlannerTurnP", 2.0);
    public static double pathPlannerTurnI = Preferences.getDouble("pathPlannerTurnI", 0.2);
    public static double pathPlannerTurnD = Preferences.getDouble("pathPlannerTurnD", 0.0);

    public static void reloadConstants() {
      pathPlannerDriveP = Preferences.getDouble("pathPlannerDriveP", 2.0);
      pathPlannerDriveI = Preferences.getDouble("pathPlannerDriveI", 0.2);
      pathPlannerDriveD = Preferences.getDouble("pathPlannerDriveD", 0.0);
      pathPlannerTurnP = Preferences.getDouble("pathPlannerTurnP", 2.0);
      pathPlannerTurnI = Preferences.getDouble("pathPlannerTurnI", 0.2);
      pathPlannerTurnD = Preferences.getDouble("pathPlannerTurnD", 0.0);
    }
  }

  public static final class PresetConstants {
    public static final String PREFERENCES_ID = "preset";
    public static double elevatorl2 =
        Preferences.getDouble(PREFERENCES_ID + "l2", 10.05); // This was 17 for testing PID
    public static double elevatorl3 = Preferences.getDouble(PREFERENCES_ID + "l3", 17);
    public static double elevatorl4 = Preferences.getDouble(PREFERENCES_ID + "l4", 19.40);
    public static double elevatorNet = Preferences.getDouble(PREFERENCES_ID, 16.486);

    public static double arml2 = Preferences.getDouble(PREFERENCES_ID + "arml2", 0.543);
    public static double arml3 = Preferences.getDouble(PREFERENCES_ID + "arml3", 0.543);
    public static double arml4 = Preferences.getDouble(PREFERENCES_ID + "arml4", 0.89);
    public static double armNet = Preferences.getDouble(PREFERENCES_ID + "armnet", 0.950); // change
    public static final double arml3Algae = 0.0;

    public static final double elevatorIntake = 0.0;
    public static final double armIntake = 0.0;

    public static void reloadConstants() {
      elevatorl2 = Preferences.getDouble(PREFERENCES_ID + "l2", 10.05);
      elevatorl3 = Preferences.getDouble(PREFERENCES_ID + "l3", 17);
      elevatorl4 = Preferences.getDouble(PREFERENCES_ID + "l4", 19.40);
      elevatorNet = Preferences.getDouble(PREFERENCES_ID, 16.486);

      arml2 = Preferences.getDouble(PREFERENCES_ID + "arml2", 0.543);
      arml3 = Preferences.getDouble(PREFERENCES_ID + "arml3", 0.543);
      arml4 = Preferences.getDouble(PREFERENCES_ID + "arml4", 0.89);
      armNet = Preferences.getDouble(PREFERENCES_ID + "armnet", 0.950);
    }
  }

  public static final class ArmConstants {
    public static final int ARM_MOTOR_ID = 44;
    public static final String PREFERENCES_ID = "arm";
    public static double positionkP = Preferences.getDouble(PREFERENCES_ID + "kP", 3.5);
    public static double positionkI = Preferences.getDouble(PREFERENCES_ID + "kI", 0.0);
    public static double positionkD = Preferences.getDouble(PREFERENCES_ID + "kD", 0.0);
   
    public static double kG = Preferences.getDouble(PREFERENCES_ID + "kG", 0.0);
    public static double kV = Preferences.getDouble(PREFERENCES_ID + "kV", 0.0);
    public static double kS = Preferences.getDouble(PREFERENCES_ID + "kS", 0.1);

    public static double velocitykP = Preferences.getDouble(PREFERENCES_ID + "kP", 3.5);
    public static double velocitykI = Preferences.getDouble(PREFERENCES_ID + "kI", 0.0);
    public static double velocitykD = Preferences.getDouble(PREFERENCES_ID + "kD", 0.0);
    public static double velocitykG = Preferences.getDouble(PREFERENCES_ID + "kG", 0.0);
    public static final int ARM_MOTOR_CURRENT_LIMIT = 40;
    // Sim constants
    public static final double GEARING = 1.0 / 1.125;
    public static final double LENGTH = Units.inchesToMeters(22.9);
    public static final double MOI =
        SingleJointedArmSim.estimateMOI(ArmConstants.LENGTH, Units.lbsToKilograms(5.3754589));
    public static final double MIN_ANGLE = Units.degreesToRadians(-90);
    public static final double MAX_ANGLE = Units.degreesToRadians(103);
    public static final double STARTING_ANGLE = Units.degreesToRadians(-90);

    public static void reloadConstants() {
      positionkP = Preferences.getDouble(PREFERENCES_ID + "kP", 0.0);
      positionkI = Preferences.getDouble(PREFERENCES_ID + "kI", 0.0);
      positionkD = Preferences.getDouble(PREFERENCES_ID + "kD", 0.0);
      kG = Preferences.getDouble(PREFERENCES_ID + "kG", 0.0);

      velocitykP = Preferences.getDouble(PREFERENCES_ID + "velocity" + "kP", 0.0);
      velocitykI = Preferences.getDouble(PREFERENCES_ID + "velocity" + "kI", 0.0);
      velocitykD = Preferences.getDouble(PREFERENCES_ID + "velocity"  + "kD", 0.0);
      kG = Preferences.getDouble(PREFERENCES_ID + "velocity" + "kG", 0.0);
  
    }
  }

  public static final class ClimbConstants {
    public static final int CLIMB_MOTOR_ID = 46;
    public static final String PREFERENCES_ID = "climb";
    public static final int CLIMB_MOTOR_CURRENT_LIMIT = 120;
    public static double CLIMB_kP = Preferences.getDouble(PREFERENCES_ID + "kP", 0.5);
    public static double CLIMB_kI = Preferences.getDouble(PREFERENCES_ID + "kI", 0.2);
    public static double CLIMB_kD = Preferences.getDouble(PREFERENCES_ID + "kD", 0.0);
    public static double CLIMB_kS = Preferences.getDouble(PREFERENCES_ID + "kS", 0.0);
    // Sim Constants
    public static final double GEARING = 100; // Update this pls don't worry bro I did
    public static final double BASE_HEIGHT = Units.inchesToMeters(6.25);
    public static final double ARM_LENGTH = Units.inchesToMeters(12.1);
    public static final double MOI =
        SingleJointedArmSim.estimateMOI(ARM_LENGTH, Units.lbsToKilograms(2.2));
    public static final double MIN_ANGLE = Units.degreesToRadians(-148.47);
    public static final double MAX_ANGLE = Units.degreesToRadians(-90);
    public static final double STARTING_ANGLE = Units.degreesToRadians(-148.47);

    public static void reloadConstants() {
      CLIMB_kP = Preferences.getDouble(PREFERENCES_ID + "kP", 0.5);
      CLIMB_kI = Preferences.getDouble(PREFERENCES_ID + "kI", 0.2);
      CLIMB_kD = Preferences.getDouble(PREFERENCES_ID + "kD", 0.0);
      CLIMB_kS = Preferences.getDouble(PREFERENCES_ID + "kS", 0.0);
    }
  }

  public static final class ElevatorConstants {
    // TODO change non sim constants
    public static final int ELEVATOR_MOTOR_ID1 = 42;
    public static final int ELEVATOR_MOTOR_ID2 = 45;
    public static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 40;
    public static final int LASERCAN_ID = 50;
    public static final String PREFERENCES_ID = "elevator";

    public static final class ElevatorPositionConstants {
      public static final double ELEVATOR_FEEDFORWARD = 0.0;
      public static double ELEVATOR_MOTOR_kP = Preferences.getDouble(PREFERENCES_ID + "kV", 0.3);
      public static double ELEVATOR_MOTOR_kI = Preferences.getDouble(PREFERENCES_ID + "kI", 0.003);
      public static double ELEVATOR_MOTOR_kD = Preferences.getDouble(PREFERENCES_ID + "kD", 0.0);
      public static double ELEVATOR_MOTOR_kG = Preferences.getDouble(PREFERENCES_ID + "kG", 0.05);
      public static double ELEVATOR_MOTOR_kV = Preferences.getDouble(PREFERENCES_ID + "kV", 0.0);
    }

    public static final class ElevatorVelocityConstants {
      public static final double ELEVATOR_FEEDFORWARD = 0.0;
      public static double ELEVATOR_MOTOR_kP =
          Preferences.getDouble(PREFERENCES_ID + "velocity" + "kV", 0.15);
      public static double ELEVATOR_MOTOR_kI =
          Preferences.getDouble(PREFERENCES_ID + "velocity" + "kI", 0.025);
      public static double ELEVATOR_MOTOR_kD =
          Preferences.getDouble(PREFERENCES_ID + "velocity" + "kD", 0.0);
      public static double ELEVATOR_MOTOR_kG =
          Preferences.getDouble(PREFERENCES_ID + "velocity" + "kG", 0.2);
      public static double ELEVATOR_MOTOR_kV =
          Preferences.getDouble(PREFERENCES_ID + "velocity" + "kV", 0.0);
    }

    public static final double LASERCAN_TO_ELEVATOR_POSITION = 1.0;
    public static final double ELEVATOR_EPSILON = 1e-2;
    // Sim constants
    public static final double GEARING = 4;
    public static final double CARRIAGE_MASS = Units.lbsToKilograms(17.1910833);
    public static final double DRUM_RADIUS = Units.inchesToMeters(1.44);
    public static final double MIN_HEIGHT = Units.inchesToMeters(1);
    public static final double MAX_HEIGHT = Units.inchesToMeters(53);
    public static final double STARTING_HEIGHT = Units.inchesToMeters(1);
    public static final double METERS_PER_ROTATION =
        2 * ElevatorConstants.DRUM_RADIUS * Math.PI / ElevatorConstants.GEARING;

    public static void reloadConstants() {
      ElevatorPositionConstants.ELEVATOR_MOTOR_kP =
          Preferences.getDouble(PREFERENCES_ID + "kP", 0.3);
      ElevatorPositionConstants.ELEVATOR_MOTOR_kI =
          Preferences.getDouble(PREFERENCES_ID + "kI", 0.03);
      ElevatorPositionConstants.ELEVATOR_MOTOR_kD =
          Preferences.getDouble(PREFERENCES_ID + "kD", 0.0);
      ElevatorPositionConstants.ELEVATOR_MOTOR_kG =
          Preferences.getDouble(PREFERENCES_ID + "kG", 0.05);
      ElevatorPositionConstants.ELEVATOR_MOTOR_kV =
          Preferences.getDouble(PREFERENCES_ID + "kV", 0.0);

      ElevatorVelocityConstants.ELEVATOR_MOTOR_kP =
          Preferences.getDouble(PREFERENCES_ID + "velocity" + "kP", 0.15);
      ElevatorVelocityConstants.ELEVATOR_MOTOR_kI =
          Preferences.getDouble(PREFERENCES_ID + "velocity" + "kI", 0.025);
      ElevatorVelocityConstants.ELEVATOR_MOTOR_kD =
          Preferences.getDouble(PREFERENCES_ID + "velocity" + "kD", 0.0);
      ElevatorVelocityConstants.ELEVATOR_MOTOR_kG =
          Preferences.getDouble(PREFERENCES_ID + "velocity" + "kG", 0.2);
      ElevatorVelocityConstants.ELEVATOR_MOTOR_kV =
          Preferences.getDouble(PREFERENCES_ID + "velocity" + "kV", 0.0);

      System.out.println("Elevator position kp: " + ElevatorPositionConstants.ELEVATOR_MOTOR_kP);
    }
  }

  public static final class EndEffectorConstants {
    public static final int EFFECTORID = 43;
    public static final int EFFECTOR_CURRENT_LIMIT = 20;
    public static final String PREFERENCES_ID = "endEffector";
    public static double EFFECTOR_KP = Preferences.getDouble(PREFERENCES_ID + "kP", 0.1);
    public static double EFFECTOR_KI = Preferences.getDouble(PREFERENCES_ID + "kI", 0.0);
    public static double EFFECTOR_KD = Preferences.getDouble(PREFERENCES_ID + "kD", 0.0);
    public static double EFFECTOR_KS = Preferences.getDouble(PREFERENCES_ID + "kS", 0.0);
    public static double EFFECTOR_KV = Preferences.getDouble(PREFERENCES_ID + "kV", 0.0);
    public static double EFFECTOR_KFF = Preferences.getDouble(PREFERENCES_ID + "kFF", 0.0);
    public static double EFFECTOR_KG = Preferences.getDouble(PREFERENCES_ID + "kG", 0.05);
    public static final int LASERCAN_1ID = -1;
    public static final int LASERCAN_2ID = -1;
    // Sim Constants
    public static final double MOI = 0.0002048478;
    public static final double GEARING = 1;

    public static void reloadConstants() {
      EFFECTOR_KP = Preferences.getDouble(PREFERENCES_ID + "kP", 0.01);
      EFFECTOR_KI = Preferences.getDouble(PREFERENCES_ID + "kI", 0.0);
      EFFECTOR_KD = Preferences.getDouble(PREFERENCES_ID + "kD", 0.0);
      EFFECTOR_KS = Preferences.getDouble(PREFERENCES_ID + "kS", 0.0);
      EFFECTOR_KV = Preferences.getDouble(PREFERENCES_ID + "kV", 0.0);
      EFFECTOR_KFF = Preferences.getDouble(PREFERENCES_ID + "kFF", 0.0);
      EFFECTOR_KG = Preferences.getDouble(PREFERENCES_ID + "kG", 0.05);
    }
  }

  public static final class GroundIntakeConstants {
    public static final class ArmConstants {
      // TODO change this
      public static final int ARM_CANID = 40; // CHANGED FOR TESTING
      public static final int ARM_CURRENT_LIMIT = 20;
      public static final String PREFERENCES_ID = "groundIntakeArm";
      public static double ARM_KP = Preferences.getDouble(PREFERENCES_ID + "kP", 0.05);
      public static double ARM_KI = Preferences.getDouble(PREFERENCES_ID + "kI", 0.0);
      public static double ARM_KD = Preferences.getDouble(PREFERENCES_ID + "kD", 0.0);
      public static double ARM_KFF = Preferences.getDouble(PREFERENCES_ID + "kFF", 0.0);
      public static double ARM_KS = Preferences.getDouble(PREFERENCES_ID + "kS", 0.0);
      public static double ARM_KV = Preferences.getDouble(PREFERENCES_ID + "kV", 0.0);
      // Sim stuff
      public static final double GEARING = 1;
      public static final double LENGTH = Units.inchesToMeters(18.5);
      public static final double MIN_ANGLE = Units.degreesToRadians(-106);
      public static final double MAX_ANGLE = Units.degreesToRadians(0);
      public static final double STARTING_ANGLE = Units.degreesToRadians(-106);
      public static final double ARM_BASE_HEIGHT = Units.inchesToMeters(6.25);
      public static final double MOI =
          SingleJointedArmSim.estimateMOI(
              GroundIntakeConstants.ArmConstants.LENGTH, Units.lbsToKilograms(4.9));

      public static void reloadConstants() {
        ARM_KP = Preferences.getDouble(PREFERENCES_ID + "kP", 0.05);
        ARM_KI = Preferences.getDouble(PREFERENCES_ID + "kI", 0.0);
        ARM_KD = Preferences.getDouble(PREFERENCES_ID + "kD", 0.0);
        ARM_KFF = Preferences.getDouble(PREFERENCES_ID + "kFF", 0.0);
        ARM_KS = Preferences.getDouble(PREFERENCES_ID + "kS", 0.0);
        ARM_KV = Preferences.getDouble(PREFERENCES_ID + "kV", 0.0);
      }
    }

    public static final class IntakeConstants {
      public static final int INTAKE_CANID = 41; // CHANGED FOR TESTING
      public static final int INTAKE_CURRENT_LIMIT = 20;
      public static final String PREFERENCES_ID = "groundIntake";
      public static double INTAKE_KP = Preferences.getDouble(PREFERENCES_ID + "kP", 0.0);
      public static double INTAKE_KI = Preferences.getDouble(PREFERENCES_ID + "kI", 0.0);
      public static double INTAKE_KD = Preferences.getDouble(PREFERENCES_ID + "kD", 0.0);
      public static double INTAKE_KFF = Preferences.getDouble(PREFERENCES_ID + "kFF", 0.0);
      public static double INTAKE_KS = Preferences.getDouble(PREFERENCES_ID + "kS", 0.0);
      public static double INTAKE_KV = Preferences.getDouble(PREFERENCES_ID + "kV", 0.0);
      // Sim stuff
      public static final double MOI = 0.0002341117;
      public static final double GEARING = 1;

      public static void reloadConstants() {
        INTAKE_KP = Preferences.getDouble(PREFERENCES_ID + "kP", 0.0);
        INTAKE_KI = Preferences.getDouble(PREFERENCES_ID + "kI", 0.0);
        INTAKE_KD = Preferences.getDouble(PREFERENCES_ID + "kD", 0.0);
        INTAKE_KFF = Preferences.getDouble(PREFERENCES_ID + "kFF", 0.0);
        INTAKE_KS = Preferences.getDouble(PREFERENCES_ID + "kS", 0.0);
        INTAKE_KV = Preferences.getDouble(PREFERENCES_ID + "kV", 0.0);
      }
    }
  }

  public static final class LEDConstants {
    // TODO change this
    public static final int LED_PWM_PORT = 0;
    public static final int LED_LENGTH = 123;
    public static final int BARGE_RANGE = 1;
    public static final Distance LED_SPACING = Meters.of(1.0 / 50.0);
  }

  public static void initPreferences() {
    String[] subsystemIdentifiers = {
      "arm",
      "armvelocity",
      "climb",
      "elevator",
      "elevatorvelocity",
      "endEffector",
      "groundIntakeArm",
      "groundIntake",
      "pathPlannerDrive",
      "pathPlannerTurn"
    };
    for (String id : subsystemIdentifiers) {
      Preferences.initDouble(id + "kP", 0);
      Preferences.initDouble(id + "kI", 0);
      Preferences.initDouble(id + "kD", 0);
      switch (id) {
        case "arm", "endEffector" -> {
          Preferences.initDouble(id + "kG", 0);
          Preferences.initDouble(id + "kV", 0);
          Preferences.initDouble(id + "kFF", 0);
          Preferences.initDouble(id + "kS", 0);
        }
        case "climb" -> {
          Preferences.initDouble(id + "kS", 0);
        }
        case "elevator", "elevatorvelocity" -> {
          Preferences.initDouble(id + "kV", 0);
          // Preferences.initDouble(id + "kFF", 0);
          // Preferences.initDouble(id + "kS", 0);
          Preferences.initDouble(id + "kG", 0);
        }
        case "groundIntake", "groundIntakeArm" -> {
          Preferences.initDouble(id + "kV", 0);
          Preferences.initDouble(id + "kFF", 0);
          Preferences.initDouble(id + "kS", 0);
        }
      }
    }
    Preferences.initDouble(PresetConstants.PREFERENCES_ID + "l2", 10.05);
    Preferences.initDouble(PresetConstants.PREFERENCES_ID + "l3", 17);
    Preferences.initDouble(PresetConstants.PREFERENCES_ID + "l4", 19.40);
    Preferences.initDouble(PresetConstants.PREFERENCES_ID + "arml2", 0.543);
    Preferences.initDouble(PresetConstants.PREFERENCES_ID + "arml3", 0.543);
    Preferences.initDouble(PresetConstants.PREFERENCES_ID + "arml4", 0.89);
    Preferences.initDouble("autoAlignTurnP", 2.0);
    Preferences.initDouble("autoAlignTurnI", 0.2);
    Preferences.initDouble("autoAlignTurnD", 0.2);
    Preferences.initDouble("autoAlignDriveP", 2.0);
    Preferences.initDouble("autoAlignDriveI", 0.2);
    Preferences.initDouble("autoAlignTurnD", 0.2);
    Preferences.initDouble("armvelocitykG", 0);
  }

  public static void reloadPreferences() {
    ArmConstants.reloadConstants();
    ClimbConstants.reloadConstants();
    ElevatorConstants.reloadConstants();
    EndEffectorConstants.reloadConstants();
    GroundIntakeConstants.ArmConstants.reloadConstants();
    GroundIntakeConstants.IntakeConstants.reloadConstants();
    PresetConstants.reloadConstants();
    VisionConstants.reloadConstants();
    DriveConstants.reloadConstants();
  }

  public class VisionConstants {

    public static double autoAligndriveP = Preferences.getDouble("autoAlignDriveP", 2.0);
    public static double autoAligndriveI = Preferences.getDouble("autoAlignDriveI", 0.2);
    public static double autoAligndriveD = Preferences.getDouble("autoAlignDriveI", 0.0);
    public static double autoAlignturnP = Preferences.getDouble("autoAlignTurnP", 2.0);
    public static double autoAlignturnI = Preferences.getDouble("autoAlignTurnI", 0.2);
    public static double autoAlignturnD = Preferences.getDouble("autoAlignDriveI", 0.0);
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

    public static void reloadConstants() {
      autoAligndriveP = Preferences.getDouble("autoAlignDriveP", 2.0);
      autoAligndriveI = Preferences.getDouble("autoAlignDriveI", 0.2);
      autoAligndriveD = Preferences.getDouble("autoAlignTurnD", 0.0);
      autoAlignturnP = Preferences.getDouble("autoAlignTurnP", 2.0);
      autoAlignturnI = Preferences.getDouble("autoAlignTurnI", 0.2);
      autoAlignturnD = Preferences.getDouble("autoAlignTurnD", 0.0);
    }
  }
}

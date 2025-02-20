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

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class ClimbConstants {
    public static final int CLIMB_MOTOR_ID = -1;
    public static final int CLIMB_MOTOR_CURRENT_LIMIT = 20;
    public static final double CLIMB_kP = 0.0;
    public static final double CLIMB_kI = 0.0;
    public static final double CLIMB_kD = 0.0;
    public static final double CLIMB_kS = 0.0;
    public static final double GEARING = 100; // Update this pls don't worry bro I did
    public static final double BASE_HEIGHT = 1;
    public static final double ARM_LENGTH = 1;
  }

  public static final class LEDConstants {
    // TODO change this
    public static final int LED_PWM_PORT = 0;
    public static final int LED_LENGTH = 300;
    public static final Distance LED_SPACING = Meters.of(1.0 / 50.0);
  }

  public static final class ArmConstants {
    public static final int ARM_MOTOR_ID = -1;
    public static final double ARM_MOTOR_KP = 0.0;
    public static final double ARM_MOTOR_KI = 0.0;
    public static final double ARM_MOTOR_KD = 0.0;
    public static final double ARM_MOTOR_KG = 0.0;
    public static final double ARM_MOTOR_KV = 0.0;
    public static final double ARM_MOTOR_KFF = 0.0;
    public static final double ARM_MOTOR_KS = 0.0;
    public static final int ARM_MOTOR_CURRENT_LIMIT = 40;
    public static final double ARM_ENCODER_POSITION_CONVERSION_FACTOR = 1.0;
    public static final double ARM_ENCODER_VELOCITY_CONVERSION_FACTOR = 1.0;
  }

  public static final class ElevatorConstants {
    // TODO change all constants
    public static final int ELEVATOR_MOTOR_ID1 = -1;
    public static final int ELEVATOR_MOTOR_ID2 = -1;
    public static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 20;
    public static final int LASERCAN_ID = -1;
    public static final double ELEVATOR_MOTOR_kP = 0.0;
    public static final double ELEVATOR_MOTOR_kI = 0.0;
    public static final double ELEVATOR_MOTOR_kD = 0.0;
    public static final double ELEVATOR_MOTOR_kG = 0.0;
    public static final double ELEVATOR_MOTOR_kV = 0.0;
    public static final double LASERCAN_TO_ELEVATOR_POSITION = 1.0;
    public static final double ELEVATOR_EPSILON = 1e-2;
    public static final double ELEVATOR_kP_LASERCAN = 0.0;
    public static final double ELEVATOR_kI_LASERCAN = 0.0;
    public static final double ELEVATOR_kD_LASERCAN = 0.0;
    // Sim constants
    public static final double GEARING = 4;
    public static final double CARRIAGE_MASS = 0;
    public static final double DRUM_RADIUS = 0;
    public static final double MIN_HEIGHT = 0;
    public static final double MAX_HEIGHT = 0;
    public static final double STARTING_HEIGHT = 0;
  }

  public static final class EndEffectorConstants {
    public static final int EFFECTORID = -1;
    public static final int EFFECTOR_CURRENT_LIMIT = 20;
    public static final double EFFECTOR_ENCODER_POSITION_CONVERSION_FACTOR = 1.0;
    public static final double EFFECTOR_ENCODER_VELOCITY_CONVERSION_FACTOR = 1.0;
    public static final double EFFECTOR_KP = 0.0;
    public static final double EFFECTOR_KI = 0.0;
    public static final double EFFECTOR_KD = 0.0;
    public static final double EFFECTOR_KS = 0.0;
    public static final double EFFECTOR_KV = 0.0;
    public static final double EFFECTOR_KFF = 0.0;
    public static final double EFFECTOR_KG = 0.0;
    public static final int LASERCAN_1ID = -1;
    public static final int LASERCAN_2ID = -1;
    // Sim Constants
    public static final double MOI = 0.0;
    public static final double GEARING = 9;
  }

  public static final class GroundIntakeConstants {
    public static final class ArmConstants{
    // TODO change this
      public static final int ARM_CANID = -1;
      public static final int ARM_CURRENT_LIMIT = 20;
      public static final int ARM_ENCODER_POSITION_CONVERSION_FACTOR = 1;
      public static final int ARM_ENCODER_VELOCITY_CONVERSION_FACTOR = 1;
      public static final double ARM_KP = 0;
      public static final double ARM_KI = 0;
      public static final double ARM_KD = 0;
      public static final double ARM_KFF = 0;
      public static final double ARM_KS = 0;
      public static final double ARM_KV = 0;
      //Sim stuff
      public static final double GEARING = 100;
      public static final double MOI = 0.0;
      public static final double LENGTH = 0.0;
      public static final double MIN_ANGLE = 0.0;
      public static final double MAX_ANGLE = 0.0;
      public static final double STARTING_ANGLE = 0.0;
    }

    public static final int INTAKE_CANID = -1;
    public static final int INTAKE_CURRENT_LIMIT = 20;
    public static final int INTAKE_ENCODER_POSITION_CONVERSION_FACTOR = 1;
    public static final int INTAKE_ENCODER_VELOCITY_CONVERSION_FACTOR = 1;
    public static final double INTAKE_KP = 0;
    public static final double INTAKE_KI = 0;
    public static final double INTAKE_KD = 0;
    public static final double INTAKE_KFF = 0;
    public static final double INTAKE_KS = 0;
    public static final double INTAKE_KV = 0;
  }
}

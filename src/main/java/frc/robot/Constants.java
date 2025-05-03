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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.elevator.ElevatorConstants;

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
    public static final double elevatorl2 =
        10.05 * ElevatorConstants.METERS_PER_ROTATION; // This was 17 for testing PID
    public static final double elevatorl3 = 17 * ElevatorConstants.METERS_PER_ROTATION;
    public static final double elevatorl4 = 19.40 * ElevatorConstants.METERS_PER_ROTATION;
    public static final double elevatorNet = 16.486 * ElevatorConstants.METERS_PER_ROTATION;
    public static final double elevatorl3Algae = 0.2 * ElevatorConstants.METERS_PER_ROTATION;

    // public static final double arml2 = 0.543;
    // public static final double arml3 = 0.543;
    public static final double arml4 = 0.78;
    public static final double armNet = 0.950 - 0.188; // change
    public static final double arml3Algae = 0.0;

    public static final double elevatorIntake = 0.0;
    public static final double armIntake = 0.0;
  }
}

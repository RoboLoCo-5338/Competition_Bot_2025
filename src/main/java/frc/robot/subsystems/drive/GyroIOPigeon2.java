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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 extends GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(
          TunerConstants.DrivetrainConstants.Pigeon2Id,
          TunerConstants.DrivetrainConstants.CANBusName);

  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final StatusSignal<Angle> roll = pigeon.getRoll();
  private final StatusSignal<Angle> pitch = pigeon.getPitch();

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> rollPositionQueue;
  private final Queue<Double> pitchPositionQueue;

  private final Queue<Double> odometryTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    yaw.setUpdateFrequency(DriveConstants.ODOMETRY_FREQUENCY);
    pitch.setUpdateFrequency(DriveConstants.ODOMETRY_FREQUENCY);
    roll.setUpdateFrequency(DriveConstants.ODOMETRY_FREQUENCY);

    yawVelocity.setUpdateFrequency(50.0);

    pigeon.optimizeBusUtilization();
    odometryTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
    rollPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getRoll());
    pitchPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getPitch());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.rotation = new Rotation3d(roll.getValue(), pitch.getValue(), yaw.getValue());

    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryTimestampQueue =
        odometryTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryPitchPositions =
        pitchPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryRollPositions =
        rollPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);

    odometryTimestampQueue.clear();
    yawPositionQueue.clear();
    rollPositionQueue.clear();
    pitchPositionQueue.clear();
  }
}

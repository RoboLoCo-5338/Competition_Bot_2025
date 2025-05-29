package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.AutoLog;

public class ElevatorIO {
  TalonFX elevatorMotor1 =
      new TalonFX(
          ElevatorConstants.ELEVATOR_MOTOR_ID1, TunerConstants.DrivetrainConstants.CANBusName);
  TalonFX elevatorMotor2 =
      new TalonFX(
          ElevatorConstants.ELEVATOR_MOTOR_ID2, TunerConstants.DrivetrainConstants.CANBusName);
  final PositionVoltage elevator1PositionRequest = new PositionVoltage(0.0);
  final VelocityVoltage elevator1VelocityRequest = new VelocityVoltage(0);
  final PositionVoltage elevator2PositionRequest = new PositionVoltage(0.0);
  final VelocityVoltage elevator2VelocityRequest = new VelocityVoltage(0);
  final VoltageOut elevatorOpenLoop = new VoltageOut(0.0);

  @AutoLog
  public static class ElevatorIOInputs {
    public double elevator1Position = 0.0;
    public double elevator1Velocity = 0.0;
    public double elevator1AppliedVolts = 0.0;
    public double elevator1CurrentAmps = 0.0;
    public boolean elevator1Connected = false;
    public double elevator1Temperature = 0.0;

    public double elevator2Position = 0.0;
    public double elevator2Velocity = 0.0;
    public double elevator2AppliedVolts = 0.0;
    public double elevator2CurrentAmps = 0.0;
    public boolean elevator2Connected = false;
    public double elevator2Temperature = 0.0;
  }

  /**
   * Updates the set of loggable inputs for the elevator subsystem. This method updates the
   * following inputs:
   *
   * <ul>
   *   <li>{@code elevator1Connected}: Whether the first elevator motor is connected
   *   <li>{@code elevator1Position}: The position of the first elevator motor in radians
   *   <li>{@code elevator1Velocity}: The velocity of the first elevator motor in radians per second
   *   <li>{@code elevator1AppliedVolts}: The voltage applied to the first elevator motor in volts
   *   <li>{@code elevator1CurrentAmps}: The current drawn by the first elevator motor in amps
   *   <li>{@code elevator2Connected}: Whether the second elevator motor is connected
   *   <li>{@code elevator2Position}: The position of the second elevator motor in radians
   *   <li>{@code elevator2Velocity}: The velocity of the second elevator motor in radians per
   *       second
   *   <li>{@code elevator2AppliedVolts}: The voltage applied to the second elevator motor in volts
   *   <li>{@code elevator2CurrentAmps}: The current drawn by the second elevator motor in amps
   * </ul>
   */
  public void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the target velocity for both elevator motors.
   *
   * <p>This method sends a velocity control request to both elevator motors to move them at the
   * specified velocity.
   *
   * @param velocity The target velocity in meters per second for the elevator motors.
   */
  public void setElevatorVelocity(double velocity) {}

  /**
   * Sets the target position for both elevator motors.
   *
   * <p>This method sends a position control request to both elevator motors to move them to the
   * specified position.
   *
   * @param position The target position in meters for the elevator motors.
   */
  public void setElevatorPosition(double position, int slot) {}

  /**
   * Gets the current measurement from the laser can sensor.
   *
   * <p>This method returns the current measurement in millimeters from the laser can sensor, or -1
   * if the measurement is invalid or not available.
   *
   * @return The current measurement in millimeters from the laser can sensor, or -1 if the
   *     measurement is invalid or not available.
   */
  public int getLaserCanMeasurement() {
    return -1;
  }

  public void elevatorOpenLoop(Voltage voltage) {}
}

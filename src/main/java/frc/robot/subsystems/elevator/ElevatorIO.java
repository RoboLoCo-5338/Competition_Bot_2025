package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
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

  @AutoLog
  public static class ElevatorIOInputs {
    public double elevator1Position = 0.0;
    public double elevator1Velocity = 0.0;
    public double elevator1AppliedVolts = 0.0;
    public double elevator1CurrentAmps = 0.0;
    public boolean elevator1Connected = false;

    public double elevator2Position = 0.0;
    public double elevator2Velocity = 0.0;
    public double elevator2AppliedVolts = 0.0;
    public double elevator2CurrentAmps = 0.0;
    public boolean elevator2Connected = false;
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
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the target velocity for both elevator motors.
   *
   * <p>This method sends a velocity control request to both elevator motors to move them at the
   * specified velocity.
   *
   * @param velocity The target velocity in meters per second for the elevator motors.
   */
  public default void setElevatorVelocity(double velocity) {}

  /**
   * Sets the target position for both elevator motors.
   *
   * <p>This method sends a position control request to both elevator motors to move them to the
   * specified position.
   *
   * @param position The target position in meters for the elevator motors.
   */
  public default void setElevatorPosition(double position) {}

  /**
   * Gets the current measurement from the laser can sensor.
   *
   * <p>This method returns the current measurement in millimeters from the laser can sensor, or -1
   * if the measurement is invalid or not available.
   *
   * @return The current measurement in millimeters from the laser can sensor, or -1 if the
   *     measurement is invalid or not available.
   */
  public default int getLaserCanMeasurement() {
    return -1;
  }

  /**
   * Gets the configuration used for the Talon FX motor controllers of the elevator subsystem.
   *
   * <p>This method returns a Talon FX configuration with the following settings:
   *
   * <ul>
   *   <li>Neutral mode: Brake
   *   <li>Gravity type: Elevator static
   *   <li>Feedback device: Integrated sensor
   *   <li>kP: {@link ElevatorConstants#ELEVATOR_MOTOR_kP}
   *   <li>kI: {@link ElevatorConstants#ELEVATOR_MOTOR_kI}
   *   <li>kD: {@link ElevatorConstants#ELEVATOR_MOTOR_kD}
   *   <li>kG: {@link ElevatorConstants#ELEVATOR_MOTOR_kG}
   *   <li>kV: {@link ElevatorConstants#ELEVATOR_MOTOR_kV}
   *   <li>Current limit: 40A (CHANGE THIS VALUE OTHERWISE TORQUE MAY BE LIMITED/TOO HIGH)
   * </ul>
   *
   * <p>These values may need to be changed based on the actual robot hardware and the desired
   * behavior of the elevator.
   *
   * @return the configuration used for the Talon FX motor controllers of the elevator subsystem
   */
  public default TalonFXConfiguration getConfiguration(int motorNum) {
    // TODO change these values
    var config = new TalonFXConfiguration();
    config.Voltage.PeakForwardVoltage = 16;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //Slot 0 is position
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kP = ElevatorConstants.ElevatorPositionConstants.ELEVATOR_MOTOR_kP;
    config.Slot0.kI = ElevatorConstants.ElevatorPositionConstants.ELEVATOR_MOTOR_kI;
    config.Slot0.kD = ElevatorConstants.ElevatorPositionConstants.ELEVATOR_MOTOR_kD;
    config.Slot0.kG = ElevatorConstants.ElevatorPositionConstants.ELEVATOR_MOTOR_kG;
    config.Slot0.kV = ElevatorConstants.ElevatorPositionConstants.ELEVATOR_MOTOR_kV;

    //Slot 1 is velocity

    config.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot1.kP = ElevatorConstants.ElevatorVelocityConstants.ELEVATOR_MOTOR_kP;
    config.Slot1.kI = ElevatorConstants.ElevatorVelocityConstants.ELEVATOR_MOTOR_kI;
    config.Slot1.kD = ElevatorConstants.ElevatorVelocityConstants.ELEVATOR_MOTOR_kD;
    config.Slot1.kG = ElevatorConstants.ElevatorVelocityConstants.ELEVATOR_MOTOR_kG;
    config.Slot1.kV = ElevatorConstants.ElevatorVelocityConstants.ELEVATOR_MOTOR_kV;


    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 120;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    var currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimitEnable = true;
    // CHANGE THIS VALUE OTHERWISE TORQUE MAY BE LIMITED/TOO HIGH
    currentConfig.StatorCurrentLimit = 120;
    config.CurrentLimits = currentConfig;
    if (motorNum == 2) config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    return config;
  }
}

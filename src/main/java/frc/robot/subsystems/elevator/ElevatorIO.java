package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setElevatorVelocity(double velocity) {}

  public default void setElevatorPosition(double position) {}

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
  public default TalonFXConfiguration getConfiguration() {
    // TODO change these values
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kP = ElevatorConstants.ELEVATOR_MOTOR_kP;
    config.Slot0.kI = ElevatorConstants.ELEVATOR_MOTOR_kI;
    config.Slot0.kD = ElevatorConstants.ELEVATOR_MOTOR_kD;
    config.Slot0.kG = ElevatorConstants.ELEVATOR_MOTOR_kG;
    config.Slot0.kV = ElevatorConstants.ELEVATOR_MOTOR_kV;

    var currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimitEnable = true;
    // CHANGE THIS VALUE OTHERWISE TORQUE MAY BE LIMITED/TOO HIGH
    currentConfig.StatorCurrentLimit = ElevatorConstants.ELEVATOR_MOTOR_CURRENT_LIMIT;
    config.CurrentLimits = currentConfig;
    return config;
  }
}

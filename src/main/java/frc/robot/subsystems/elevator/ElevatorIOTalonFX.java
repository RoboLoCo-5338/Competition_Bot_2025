package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

public class ElevatorIOTalonFX extends ElevatorIO {

  private final StatusSignal<Angle> elevator1Position;
  private final StatusSignal<AngularVelocity> elevator1Velocity;
  private final StatusSignal<Voltage> elevator1AppliedVolts;
  private final StatusSignal<Current> elevator1Current;
  private final StatusSignal<Angle> elevator2Position;
  private final StatusSignal<AngularVelocity> elevator2Velocity;
  private final StatusSignal<Voltage> elevator2AppliedVolts;
  private final StatusSignal<Current> elevator2Current;
  private final StatusSignal<Temperature> elevator1Temperature;
  private final StatusSignal<Temperature> elevator2Temperature;
  private final StatusSignal<Integer> elevator1Version;
  private final StatusSignal<Integer> elevator2Version;

  private final Debouncer elevator1ConnectedDebounce = new Debouncer(0.5);
  private final Debouncer elevator2ConnectedDebounce = new Debouncer(0.5);

  final PositionVoltage elevator1PositionRequest = new PositionVoltage(0.0);
  final VelocityVoltage elevator1VelocityRequest = new VelocityVoltage(0);
  final PositionVoltage elevator2PositionRequest = new PositionVoltage(0.0);
  final VelocityVoltage elevator2VelocityRequest = new VelocityVoltage(0);
  final VoltageOut elevatorOpenLoop = new VoltageOut(0.0);

  TalonFX elevatorMotor1 =
      new TalonFX(
          ElevatorConstants.ELEVATOR_MOTOR_ID1, TunerConstants.DrivetrainConstants.CANBusName);
  TalonFX elevatorMotor2 =
      new TalonFX(
          ElevatorConstants.ELEVATOR_MOTOR_ID2, TunerConstants.DrivetrainConstants.CANBusName);

  public ElevatorIOTalonFX() {
    elevatorMotor1.getConfigurator().apply(getConfiguration(1));

    elevatorMotor2.getConfigurator().apply(getConfiguration(2));
    elevatorMotor1.setPosition(0);
    elevatorMotor2.setPosition(0);

    // get status signals
    elevator1Position = elevatorMotor1.getPosition();
    elevator1Velocity = elevatorMotor1.getVelocity();
    elevator1AppliedVolts = elevatorMotor1.getMotorVoltage();
    elevator1Current = elevatorMotor1.getStatorCurrent();
    elevator1Temperature = elevatorMotor1.getDeviceTemp();
    elevator1Version = elevatorMotor1.getVersion();

    elevator2Position = elevatorMotor2.getPosition();
    elevator2Velocity = elevatorMotor2.getVelocity();
    elevator2AppliedVolts = elevatorMotor2.getMotorVoltage();
    elevator2Current = elevatorMotor2.getStatorCurrent();
    elevator2Temperature = elevatorMotor2.getDeviceTemp();
    elevator2Version = elevatorMotor2.getVersion();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                elevator1Position,
                elevator1Velocity,
                elevator1AppliedVolts,
                elevator1Current,
                elevator2Position,
                elevator2Velocity,
                elevator2AppliedVolts,
                elevator2Current,
                elevator1Temperature,
                elevator2Temperature));
    ParentDevice.optimizeBusUtilizationForAll(elevatorMotor1, elevatorMotor2);
    elevatorMotor2.setControl(new StrictFollower(elevatorMotor1.getDeviceID()));
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
  public TalonFXConfiguration getConfiguration(int motorNum) {
    var config = new TalonFXConfiguration();
    config.Voltage.PeakForwardVoltage = 16;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Slot 0 is position
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kP = ElevatorConstants.ElevatorPositionConstants.ELEVATOR_MOTOR_kP;
    config.Slot0.kI = ElevatorConstants.ElevatorPositionConstants.ELEVATOR_MOTOR_kI;
    config.Slot0.kD = ElevatorConstants.ElevatorPositionConstants.ELEVATOR_MOTOR_kD;
    config.Slot0.kG = ElevatorConstants.ElevatorPositionConstants.ELEVATOR_MOTOR_kG;
    config.Slot0.kV = ElevatorConstants.ElevatorPositionConstants.ELEVATOR_MOTOR_kV;

    // Slot 1 is velocity

    config.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot1.kP = ElevatorConstants.ElevatorVelocityConstants.ELEVATOR_MOTOR_kP;
    config.Slot1.kI = ElevatorConstants.ElevatorVelocityConstants.ELEVATOR_MOTOR_kI;
    config.Slot1.kD = ElevatorConstants.ElevatorVelocityConstants.ELEVATOR_MOTOR_kD;
    config.Slot1.kG = ElevatorConstants.ElevatorVelocityConstants.ELEVATOR_MOTOR_kG;
    config.Slot1.kV = ElevatorConstants.ElevatorVelocityConstants.ELEVATOR_MOTOR_kV;

    config.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot2.kP = ElevatorConstants.ElevatorStowPresetConstants.ELEVATOR_MOTOR_kP;
    config.Slot2.kI = ElevatorConstants.ElevatorStowPresetConstants.ELEVATOR_MOTOR_kI;
    config.Slot2.kD = ElevatorConstants.ElevatorStowPresetConstants.ELEVATOR_MOTOR_kD;
    config.Slot2.kG = ElevatorConstants.ElevatorStowPresetConstants.ELEVATOR_MOTOR_kG;
    config.Slot2.kV = ElevatorConstants.ElevatorStowPresetConstants.ELEVATOR_MOTOR_kV;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 22.7;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.1;

    var currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimitEnable = true;
    // CHANGE THIS VALUE OTHERWISE TORQUE MAY BE LIMITED/TOO HIGH
    currentConfig.StatorCurrentLimit = 140;
    config.CurrentLimits = currentConfig;
    if (motorNum == 2) config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    return config;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var motor1Status =
        BaseStatusSignal.refreshAll(
            elevator1Position, elevator1Velocity, elevator1Current, elevator1AppliedVolts);

    var motor2Status =
        BaseStatusSignal.refreshAll(
            elevator2Position, elevator2Velocity, elevator2Current, elevator2AppliedVolts);

    inputs.position =
        (elevator1Position.getValueAsDouble() + elevator2Position.getValueAsDouble()) / 2;
    inputs.velocity =
        (elevator1Velocity.getValueAsDouble() + elevator2Velocity.getValueAsDouble()) / 2;

    inputs.elevator1Connected = elevator1ConnectedDebounce.calculate(motor1Status.isOK());
    inputs.elevator1Position = elevator1Position.getValueAsDouble();
    inputs.elevator1Velocity = Units.rotationsToRadians(elevator1Velocity.getValueAsDouble());
    inputs.elevator1AppliedVolts = elevator1AppliedVolts.getValueAsDouble();
    inputs.elevator1CurrentAmps = elevator1Current.getValueAsDouble();
    inputs.elevator1Temperature = elevator1Temperature.getValueAsDouble();

    inputs.elevator2Connected = elevator2ConnectedDebounce.calculate(motor2Status.isOK());
    inputs.elevator2Position = elevator2Position.getValueAsDouble();
    inputs.elevator2Velocity = Units.rotationsToRadians(elevator2Velocity.getValueAsDouble());
    inputs.elevator2AppliedVolts = elevator2AppliedVolts.getValueAsDouble();
    inputs.elevator2CurrentAmps = elevator2Current.getValueAsDouble();
    inputs.elevator2Temperature = elevator2Temperature.getValueAsDouble();
  }

  @Override
  public void setElevatorPosition(double position, int slot) {
    // elevator1PositionRequest.FeedForward =
    //     ElevatorConstants.ElevatorPositionConstants.ELEVATOR_FEEDFORWARD;
    // elevator2PositionRequest.FeedForward =
    //     ElevatorConstants.ElevatorPositionConstants.ELEVATOR_FEEDFORWARD;

    elevatorMotor1.setControl(elevator1PositionRequest.withPosition(position).withSlot(slot));
    System.out.println(slot);
  }

  @Override
  public void setElevatorVelocity(double velocity) {
    elevator1VelocityRequest.FeedForward =
        ElevatorConstants.ElevatorVelocityConstants.ELEVATOR_FEEDFORWARD;
    elevatorMotor1.setControl(elevator1VelocityRequest.withVelocity(velocity).withSlot(1));
  }

  @Override
  public void elevatorOpenLoop(Voltage voltage) {
    elevatorMotor1.setControl(elevatorOpenLoop.withOutput(voltage));
  }

  @Override
  public void close() {
    elevatorMotor1.close();
    elevatorMotor2.close();
  }
}

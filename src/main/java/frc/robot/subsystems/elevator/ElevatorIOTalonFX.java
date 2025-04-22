package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {

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
                elevator2Current));
    ParentDevice.optimizeBusUtilizationForAll(elevatorMotor1, elevatorMotor2);
    elevatorMotor2.setControl(new StrictFollower(elevatorMotor1.getDeviceID()));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var motor1Status =
        BaseStatusSignal.refreshAll(
            elevator1Position, elevator1Velocity, elevator1Current, elevator1AppliedVolts);

    var motor2Status =
        BaseStatusSignal.refreshAll(
            elevator2Position, elevator2Velocity, elevator2Current, elevator2AppliedVolts);

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
  public void elevatorOpenLoop(Voltage voltage){
    elevatorMotor1.setVoltage(voltage.magnitude());
  }
}

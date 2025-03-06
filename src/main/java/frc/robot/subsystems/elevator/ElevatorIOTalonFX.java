package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final StatusSignal<Angle> elevator1Position;
  private final StatusSignal<AngularVelocity> elevator1Velocity;
  private final StatusSignal<Voltage> elevator1AppliedVolts;
  private final StatusSignal<Current> elevator1Current;
  private final StatusSignal<Angle> elevator2Position;
  private final StatusSignal<AngularVelocity> elevator2Velocity;
  private final StatusSignal<Voltage> elevator2AppliedVolts;
  private final StatusSignal<Current> elevator2Current;

  private final Debouncer elevator1ConnectedDebounce = new Debouncer(0.5);
  private final Debouncer elevator2ConnectedDebounce = new Debouncer(0.5);

  private final LaserCan lc;

  public ElevatorIOTalonFX() {
    elevatorMotor1.getConfigurator().apply(getConfiguration(1));

    // TODO does this need to be inverted? idk bro does it? It does
    elevatorMotor2.getConfigurator().apply(getConfiguration(2));
    elevatorMotor1.setPosition(0);
    elevatorMotor2.setPosition(0);

    // get status signals
    elevator1Position = elevatorMotor1.getPosition();
    elevator1Velocity = elevatorMotor1.getVelocity();
    elevator1AppliedVolts = elevatorMotor1.getMotorVoltage();
    elevator1Current = elevatorMotor1.getStatorCurrent();

    elevator2Position = elevatorMotor2.getPosition();
    elevator2Velocity = elevatorMotor2.getVelocity();
    elevator2AppliedVolts = elevatorMotor2.getMotorVoltage();
    elevator2Current = elevatorMotor2.getStatorCurrent();

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

    lc = new LaserCan(ElevatorConstants.LASERCAN_ID);
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      // lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      // lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {

      System.out.println("Configuration failed! " + e);
    }
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
    inputs.elevator1Position = Units.rotationsToRadians(elevator1Position.getValueAsDouble());
    inputs.elevator1Velocity = Units.rotationsToRadians(elevator1Velocity.getValueAsDouble());
    inputs.elevator1AppliedVolts = elevator1AppliedVolts.getValueAsDouble();
    inputs.elevator1CurrentAmps = elevator1Current.getValueAsDouble();

    inputs.elevator2Connected = elevator2ConnectedDebounce.calculate(motor2Status.isOK());
    inputs.elevator2Position = Units.rotationsToRadians(elevator2Position.getValueAsDouble());
    inputs.elevator2Velocity = Units.rotationsToRadians(elevator2Velocity.getValueAsDouble());
    inputs.elevator2AppliedVolts = elevator2AppliedVolts.getValueAsDouble();
    inputs.elevator2CurrentAmps = elevator2Current.getValueAsDouble();
  }

  @Override
  public void setElevatorPosition(double position) {
    elevator1PositionRequest.FeedForward = ElevatorConstants.ELEVATOR_FEEDFORWARD;
    elevator2PositionRequest.FeedForward = ElevatorConstants.ELEVATOR_FEEDFORWARD;
    elevator1PositionRequest.Velocity = 5;
    elevator2PositionRequest.Velocity = 5;
    elevator1PositionRequest.OverrideBrakeDurNeutral = true;
    elevator2PositionRequest.OverrideBrakeDurNeutral = true;
    elevatorMotor1.setControl(elevator1PositionRequest.withPosition(position));
    elevatorMotor2.setControl(elevator2PositionRequest.withPosition(position));
  }

  @Override
  public void setElevatorVelocity(double velocity) {
    elevator1VelocityRequest.FeedForward = ElevatorConstants.ELEVATOR_FEEDFORWARD;
    elevator2VelocityRequest.FeedForward = ElevatorConstants.ELEVATOR_FEEDFORWARD;
    elevatorMotor1.setControl(elevator1VelocityRequest.withVelocity(velocity));
    elevatorMotor2.setControl(elevator2VelocityRequest.withVelocity(velocity));
  }

  @Override
  public int getLaserCanMeasurement() {
    Measurement m = lc.getMeasurement();
    if (m != null && m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return m.distance_mm;
    } else {
      return -1;
    }
  }
}

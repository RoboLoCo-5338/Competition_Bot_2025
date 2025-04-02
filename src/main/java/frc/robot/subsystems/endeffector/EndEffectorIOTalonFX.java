package frc.robot.subsystems.endeffector;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class EndEffectorIOTalonFX implements EndEffectorIO {

  private final StatusSignal<AngularVelocity> endEffectorVelocity;
  private final StatusSignal<Voltage> endEffectorAppliedVolts;
  private final StatusSignal<Current> endEffectorCurrent;
  private final StatusSignal<Temperature> endEffectorTemperature;
  private final StatusSignal<Integer> endEffectorVersion;

  private final Debouncer effectorDebouncer = new Debouncer(0.5);

  private final LaserCan LcEffector1;
  private final LaserCan LcEffector2;

  public EndEffectorIOTalonFX() {

    endEffectorVelocity = endEffectorMotor.getVelocity();
    endEffectorAppliedVolts = endEffectorMotor.getMotorVoltage();
    endEffectorCurrent = endEffectorMotor.getStatorCurrent();
    endEffectorTemperature = endEffectorMotor.getDeviceTemp();
    endEffectorVersion = endEffectorMotor.getVersion();

    endEffectorMotor.getConfigurator().apply(getEndEffectorConfiguration());

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, endEffectorVelocity, endEffectorAppliedVolts, endEffectorCurrent));

    ParentDevice.optimizeBusUtilizationForAll(endEffectorMotor);

    LcEffector1 = new LaserCan(EndEffectorConstants.LASERCAN_1ID);
    LcEffector2 = new LaserCan(EndEffectorConstants.LASERCAN_2ID);
    try {
      LcEffector1.setRangingMode(LaserCan.RangingMode.SHORT);
      LcEffector2.setRangingMode(LaserCan.RangingMode.SHORT);
      LcEffector1.setRegionOfInterest(
          new LaserCan.RegionOfInterest(2, 2, 2, 2)); // needs to be changed
      LcEffector2.setRegionOfInterest(
          new LaserCan.RegionOfInterest(2, 2, 2, 2)); // also needs to be changed
      LcEffector1.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
      LcEffector2.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);

    } catch (Exception e) {
      System.out.println("Error: " + e);
    }
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    var motor1Status =
        BaseStatusSignal.refreshAll(
            endEffectorVelocity, endEffectorCurrent, endEffectorAppliedVolts);

    inputs.endEffectorConnected = effectorDebouncer.calculate(motor1Status.isOK());
    inputs.endEffectorDistance1 = getLaserCanMeasurement1();
    inputs.endEffectorDistance2 = getLaserCanMeasurement2();
    inputs.endEffectorVelocity = Units.rotationsToRadians(endEffectorVelocity.getValueAsDouble());
    inputs.endEffectorAppliedVolts = endEffectorAppliedVolts.getValueAsDouble();
    inputs.endEffectorCurrentAmps = endEffectorCurrent.getValueAsDouble();
    inputs.endEffectorTemperature = endEffectorTemperature.getValueAsDouble();
  }

  @Override
  public void setEndEffectorVelocity(double velocity) {
    endEffectorMotor.setControl(
        endEffectorVelocityRequest.withVelocity(velocity * EndEffectorConstants.GEARING));
  }

  @Override
  public void setEndEffectorSpeed(double speed) {
    endEffectorMotor.set(speed);
  }

  @Override
  public int getLaserCanMeasurement1() {
    Measurement m1 = LcEffector1.getMeasurement();
    if (m1 != null && m1.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return m1.distance_mm;
    } else {
      if (m1.status == LaserCan.LASERCAN_STATUS_WEAK_SIGNAL) {
        return 200;
      }
      return -1;
    }
  }

  @Override
  public int getLaserCanMeasurement2() {
    Measurement m2 = LcEffector2.getMeasurement();
    if (m2 != null && m2.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return m2.distance_mm;
    } else {
      if (m2.status == LaserCan.LASERCAN_STATUS_WEAK_SIGNAL) {
        return 200;
      }
      return -1;
    }
  }
}

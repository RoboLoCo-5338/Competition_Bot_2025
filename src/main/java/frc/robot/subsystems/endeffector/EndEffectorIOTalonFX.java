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
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOTalonFX implements EndEffectorIO {

  private final StatusSignal<AngularVelocity> endEffectorVelocity;
  private final StatusSignal<Voltage> endEffectorAppliedVolts;
  private final StatusSignal<Current> endEffectorCurrent;

  private final Debouncer effectorDebouncer = new Debouncer(0.5);

  private final LaserCan LcEffector1;
  private final LaserCan LcEffector2; // why is it screaming at me here
  // aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
  // bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
  // have you tried simply being more skilled
  // - david
  public EndEffectorIOTalonFX() {

    endEffectorVelocity = endEffectorMotor.getVelocity();
    endEffectorAppliedVolts = endEffectorMotor.getMotorVoltage();
    endEffectorCurrent = endEffectorMotor.getStatorCurrent();

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

    inputs.endEffectorVelocity = Units.rotationsToRadians(endEffectorVelocity.getValueAsDouble());
    inputs.endEffectorAppliedVolts = endEffectorAppliedVolts.getValueAsDouble();
    inputs.endEffectorCurrentAmps = endEffectorCurrent.getValueAsDouble();

    Measurement m = LcEffector1.getMeasurement();
    if (m != null && m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.endEffectorDistance1 = m.distance_mm;
    } else {
      inputs.endEffectorDistance1 = -1;
    }
    m = LcEffector2.getMeasurement();
    if (m != null && m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.endEffectorDistance2 = m.distance_mm;
    } else {
      inputs.endEffectorDistance2 = -1;
    }
  }

  @Override
  public void setEndEffectorVelocity(double velocity) {
    endEffectorMotor.setControl(
        endEffectorVelocityRequest.withVelocity(velocity * EndEffectorConstants.GEARING));
  }

  @Override
  public void setEndEffectorSpeed(double speed){
    endEffectorMotor.set(speed);
  }
}

package frc.robot.subsystems.climb;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ClimbIOSim implements ClimbIO {
  @AutoLogOutput(key = "Climb/Mechanism")
  LoggedMechanism2d mechanism = new LoggedMechanism2d(5, 5);

  LoggedMechanismRoot2d root = mechanism.getRoot("climb", 2.5, 0);
  LoggedMechanismLigament2d m_climbBase = root.append(new LoggedMechanismLigament2d("base", 1, 90));
  LoggedMechanismLigament2d m_climbArm =
      m_climbBase.append(new LoggedMechanismLigament2d("arm", 1, 90));

  SingleJointedArmSim physicSim =
      new SingleJointedArmSim(DCMotor.getFalcon500(1), 0, 0, 0, 0, 0, false, 0);
  TalonFXSimState simMotor = climbMotor.getSimState();

  private final StatusSignal<Angle> climbPosition;
  private final StatusSignal<AngularVelocity> climbVelocity;
  private final StatusSignal<Voltage> climbAppliedVolts;
  private final StatusSignal<Current> climbCurrent;

  private final Debouncer climbConnected = new Debouncer(0.5);

  private final PositionVoltage climbPositionRequest = new PositionVoltage(0.0);
  private final VelocityVoltage climbVelocityRequest = new VelocityVoltage(0.0);

  public ClimbIOSim() {
    climbMotor.getConfigurator().apply(getConfiguration());

    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());

    climbPosition = climbMotor.getPosition();
    climbVelocity = climbMotor.getVelocity();
    climbAppliedVolts = climbMotor.getMotorVoltage();
    climbCurrent = climbMotor.getStatorCurrent();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, climbPosition, climbVelocity, climbAppliedVolts, climbCurrent));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    var climbStatus =
        BaseStatusSignal.refreshAll(
            climbPosition, climbAppliedVolts, climbAppliedVolts, climbCurrent);

    inputs.climbConnected = climbConnected.calculate(climbStatus.isOK());
    inputs.climbAppliedVolts = climbAppliedVolts.getValueAsDouble();
    inputs.climbCurrentAmps = climbCurrent.getValueAsDouble();
    inputs.climbPosition = Units.rotationsToRadians(climbPosition.getValueAsDouble());
    inputs.climbVelocityRadPerSec = Units.rotationsToRadians(climbPosition.getValueAsDouble());

    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_climbArm.setAngle(Units.rotationsToDegrees(climbPosition.getValueAsDouble())+90);
    physicSim.update(0.02);
  }

  @Override
  public void setClimbVelocity(double velocity) {
    System.out.println("runs");
    
  }

  @Override
  public void setClimbPosition(double position) {
    climbMotor.setControl(climbPositionRequest.withPosition(position));
  }
}

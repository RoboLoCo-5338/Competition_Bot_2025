package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.AutoLog;

public class EndEffectorIO {
  public final TalonFX endEffectorMotor =
      new TalonFX(EndEffectorConstants.EFFECTORID, TunerConstants.DrivetrainConstants.CANBusName);
  final VelocityVoltage endEffectorVelocityRequest = new VelocityVoltage(0.0);
  final VoltageOut endEffectorOpenLoop = new VoltageOut(0.0);

  @AutoLog
  public static class EndEffectorIOInputs {
    public double endEffectorVelocity = 0.0;
    public double endEffectorAppliedVolts = 0.0;
    public double endEffectorCurrentAmps = 0.0;
    public boolean endEffectorConnected = false;
    public double endEffectorDistance1 = -1;
    public double endEffectorDistance2 = -1;
    public double endEffectorTemperature = 0.0;
    public double endEffectorPosition = 0.0;
  }

  public void updateInputs(EndEffectorIOInputs inputs) {}

  public void setEndEffectorVelocity(double velocity) {}

  public void setEndEffectorSpeed(double speed) {}

  public int getLaserCanMeasurement1() {
    return -1;
  }

  public int getLaserCanMeasurement2() {
    return -1;
  }

  public void endEffectorOpenLoop(Voltage voltage) {}
}

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  public final TalonFX endEffectorMotor =
      new TalonFX(EndEffectorConstants.EFFECTORID, TunerConstants.DrivetrainConstants.CANBusName);
  final VelocityVoltage endEffectorVelocityRequest = new VelocityVoltage(0.0);

  @AutoLog
  public static class EndEffectorIOInputs {
    public double endEffectorVelocity = 0.0;
    public double endEffectorAppliedVolts = 0.0;
    public double endEffectorCurrentAmps = 0.0;
    public boolean endEffectorConnected = false;
    public double endEffectorDistance1 = -1;
    public double endEffectorDistance2 = -1;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setEndEffectorVelocity(double velocity) {}

  public default TalonFXConfiguration getEndEffectorConfiguration() {
    // TODO change these values
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // TODO requres certain canges - remember
    // IF NOT CHANGED WILL NOT WORK
    // mfw i need to do canges 🤯
    config.Slot0.kP = EndEffectorConstants.EFFECTOR_KP;
    config.Slot0.kI = EndEffectorConstants.EFFECTOR_KI;
    config.Slot0.kD = EndEffectorConstants.EFFECTOR_KD;
    config.Slot0.kG = EndEffectorConstants.EFFECTOR_KG;
    config.Slot0.kV = EndEffectorConstants.EFFECTOR_KV;

    var currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimit = EndEffectorConstants.EFFECTOR_CURRENT_LIMIT;
    config.CurrentLimits = currentConfig;
    return config;
  }

  public default void setEndEffectorSpeed(double speed){
    
  }
}

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb2 extends SubsystemBase {
  TalonFX m_kraken;

  public Climb2() {
    m_kraken = new TalonFX(13);
    m_kraken.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setForward(double speed) {
    m_kraken.set(speed);
  }

  public void setBackward(double speed) {
    m_kraken.set(-speed);
  }

  public void stop() {
    m_kraken.set(0);
  }
}

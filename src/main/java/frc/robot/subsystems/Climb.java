package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climb extends SubsystemBase {
    TalonFX m_kraken;

    public Climb() {
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

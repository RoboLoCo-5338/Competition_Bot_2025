package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import java.util.ArrayList;

public interface MusicSubsystem { // Any subsystem that can play music(ctre motors)
  public ArrayList<TalonFX> getModules(); // Must have the getModules method
}

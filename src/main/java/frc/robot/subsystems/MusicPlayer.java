package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.ArrayList;

public class MusicPlayer {
  Orchestra m_orchestra;

  public MusicPlayer(MusicSubsystem... subsystems) {
    m_orchestra = new Orchestra(getInstruments(subsystems), "Bus.chrp");
  }

  public ArrayList<ParentDevice> getInstruments(MusicSubsystem... subsystems) {
    ArrayList<ParentDevice> instruments = new ArrayList<ParentDevice>();
    for (MusicSubsystem subsystem : subsystems) {
      for (TalonFX module : subsystem.getModules()) {
        instruments.add(module);
      }
    }
    System.out.println(instruments);
    return instruments;
  }

  public void toggleMusic() {
    StatusCode status;
    if (m_orchestra.isPlaying()) {
      status = m_orchestra.pause();
    } else {
      status = m_orchestra.play();
    }
  }
}

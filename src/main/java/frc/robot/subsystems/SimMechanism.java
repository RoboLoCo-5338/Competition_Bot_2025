package frc.robot.subsystems;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;

public class SimMechanism {
  public static ArrayList<SimMechanism> MECHANISMS = new ArrayList<SimMechanism>();

  public static void updateBatteryVoltages() {
    ArrayList<Double> currents = new ArrayList<Double>();
    for (SimMechanism m : MECHANISMS) {
      for (double current : m.getCurrents()) {
        currents.add(current);
      }
    }
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            currents.stream().mapToDouble(Double::doubleValue).toArray()));
  }

  public double[] getCurrents() {
    return new double[] {0.0}; // Default implementation, should be overridden by subclasses
  }
}

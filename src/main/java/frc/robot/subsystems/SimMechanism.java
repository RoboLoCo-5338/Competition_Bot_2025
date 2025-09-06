package frc.robot.subsystems;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;

public interface SimMechanism {
  static final ArrayList<SimMechanism> MECHANISMS = new ArrayList<SimMechanism>();

  /**
   * Registers the SimMechanism to have its current draw included in battery voltage calculations.
   */
  public default void initSimVoltage() {
    MECHANISMS.add(this);
  }

  /** Updates the battery voltage based on the current draw of all registered SimMechanisms. */
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

  /**
   * Gets the current draw of the motor
   *
   * @return
   */
  public abstract double[] getCurrents();
}

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public abstract class SimMechanism {
    static final ArrayList<SimMechanism> MECHANISMS = new ArrayList<SimMechanism>();
    public SimMechanism(){
        MECHANISMS.add(this);
    }
    public static void updateBatteryVoltages(){
        ArrayList<Double> currents = new ArrayList<Double>();
        for(SimMechanism m: MECHANISMS){
            for(double current: m.getCurrents()){
                currents.add(current);
            }
        }
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(currents.stream().mapToDouble(Double::doubleValue).toArray()));
    }
    public abstract double[] getCurrents();
}

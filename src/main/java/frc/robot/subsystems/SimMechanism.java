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
        double[] currents = new double[MECHANISMS.size()];
        for(int i=0; i<currents.length; i++){
            currents[i]=MECHANISMS.get(i).getCurrent();
        }
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(currents));
    }
    public abstract double getCurrent(); 
}

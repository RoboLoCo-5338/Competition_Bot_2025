package frc.robot.subsystems;

import java.util.ArrayList;

public class mechanismIO {
    private static ArrayList<mechanismIO> mechanisms;

    public mechanismIO() {
        mechanisms.add(this);
        // SmartDashboard.putBoolean(this.getClass().getSimpleName(), false);
    }

    public static void addMechanism(mechanismIO mechanism) {
        mechanisms.add(mechanism);
    }

    public static ArrayList<mechanismIO> getMechanisms() {
        return mechanisms;
    }
}

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final Alert elevator1DisconnectedAlert = new Alert(
        "Elevator motor 1 disconnected", AlertType.kError);
        private final Alert elevator2DisconnectedAlert = new Alert(
        "Elevator motor 1 disconnected", AlertType.kError);

    public Elevator(ElevatorIO io){
        this.io = io;

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        elevator1DisconnectedAlert.set(!inputs.elevator1Connected && Constants.currentMode != Mode.SIM);
        elevator2DisconnectedAlert.set(!inputs.elevator1Connected && Constants.currentMode != Mode.SIM);
    }


    
    
}

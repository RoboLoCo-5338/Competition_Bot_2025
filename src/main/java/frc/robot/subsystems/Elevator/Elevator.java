package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Mode;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double prevError = 0;
    private double integral = 0;
    private double error;
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

    public Command setElevatorPositionLaserCan(double position){
      
        return new FunctionalCommand(null,  null, (interrupted) -> io.setElevatorVelocity(0.0), ()-> Math.abs(convertLaserCanToElevatorPosition(io.getLaserCanMeasurement())-position)<ElevatorConstants.ELEVATOR_EPSILON);
    }

    public Command setElevatorPosition(double position){
        return new InstantCommand(() -> io.setElevatorPosition(position));
    }

    public double convertLaserCanToElevatorPosition(int measurement){

        return measurement*ElevatorConstants.LASERCAN_TO_ELEVATOR_POSITION;
    }

    private void elevatorPID(double position){
        double curPosition = convertLaserCanToElevatorPosition(io.getLaserCanMeasurement());
        error = position - curPosition;
        integral += error;
        double derivative = error - prevError;
        double output = ElevatorConstants.ELEVATOR_kP_LASERCAN * error + ElevatorConstants.ELEVATOR_kI_LASERCAN * integral + ElevatorConstants.ELEVATOR_kD_LASERCAN * derivative;
       
        io.setElevatorVelocity(output);
    }

    public Command moveElevatorLaserCan(double position){
        return new FunctionalCommand(
                ()->{integral =0; prevError = 0; error=0; io.setElevatorVelocity(0.0);},
                ()-> elevatorPID(position),
                (interrupted) -> io.setElevatorVelocity(0.0),
                ()-> Math.abs(error) < ElevatorConstants.ELEVATOR_EPSILON
            );
    }







    
    
}

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSimConstants;
import frc.robot.subsystems.sim.SimMechanism;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorIOSim extends ElevatorIOTalonFX implements SimMechanism {
  TalonFXSimState motor1Sim = elevatorMotor1.getSimState();
  TalonFXSimState motor2Sim = elevatorMotor2.getSimState();
  ElevatorSim physicsSim =
      new ElevatorSim(
          DCMotor.getFalcon500(2),
          ElevatorConstants.GEARING,
          ElevatorSimConstants.CARRIAGE_MASS,
          ElevatorSimConstants.DRUM_RADIUS,
          ElevatorSimConstants.MIN_HEIGHT,
          ElevatorSimConstants.MAX_HEIGHT,
          true,
          ElevatorSimConstants.STARTING_HEIGHT);

  @AutoLogOutput(key = "Elevator/Mechanism")
  LoggedMechanism2d elevatorDrawn =
      new LoggedMechanism2d(Constants.ROBOT_LENGTH * 2, ElevatorSimConstants.MAX_HEIGHT * 2);

  LoggedMechanismRoot2d root =
      elevatorDrawn.getRoot("elevator", Constants.ROBOT_LENGTH, Constants.FLOOR_TO_MECHANISM);
  LoggedMechanismLigament2d elevator =
      root.append(new LoggedMechanismLigament2d("stage", ElevatorSimConstants.STARTING_HEIGHT, 90));

  public ElevatorIOSim() {
    super();
    motor2Sim.Orientation = ChassisReference.Clockwise_Positive;
    initSimVoltage();
  }

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    motor1Sim.setSupplyVoltage(RobotController.getBatteryVoltage());
    motor2Sim.setSupplyVoltage(RobotController.getBatteryVoltage());

    physicsSim.setInputVoltage((motor1Sim.getMotorVoltage() + motor2Sim.getMotorVoltage()) / 2);

    System.out.println(
        physicsSim.getPositionMeters() / ElevatorConstants.METERS_PER_ROTATION
            + " "
            + inputs.position);

    Logger.recordOutput("elevator1Position", physicsSim.getPositionMeters());
    Logger.recordOutput("elevator1Velocity", physicsSim.getVelocityMetersPerSecond());
    Logger.recordOutput("elevator1AppliedVolts", motor1Sim.getMotorVoltage());
    Logger.recordOutput("elevator1CurrentAmps", motor2Sim.getSupplyCurrent());

    Logger.recordOutput("elevator2Position", physicsSim.getPositionMeters());
    Logger.recordOutput("elevator2Velocity", physicsSim.getVelocityMetersPerSecond());
    Logger.recordOutput("elevator2AppliedVolts", motor2Sim.getMotorVoltage());
    Logger.recordOutput("elevator2CurrentAmps", motor2Sim.getSupplyCurrent());

    physicsSim.update(0.02);

    motor1Sim.setRawRotorPosition(
        physicsSim.getPositionMeters() / ElevatorConstants.METERS_PER_ROTATION);
    motor2Sim.setRawRotorPosition(
        physicsSim.getPositionMeters() / ElevatorConstants.METERS_PER_ROTATION);
    motor1Sim.setRotorVelocity(
        physicsSim.getVelocityMetersPerSecond() / ElevatorConstants.METERS_PER_ROTATION);
    motor2Sim.setRotorVelocity(
        physicsSim.getVelocityMetersPerSecond() / ElevatorConstants.METERS_PER_ROTATION);

    elevator.setLength(physicsSim.getPositionMeters());

    super.updateInputs(inputs);
  }

  @Override
  public double[] getCurrents() {
    return new double[] {physicsSim.getCurrentDrawAmps()};
  }

  public LoggedMechanismLigament2d getLigamentEnd() {
    return elevator;
  }

  @Override
  public void close() {
    super.close();
    elevatorDrawn.close();
  }
}

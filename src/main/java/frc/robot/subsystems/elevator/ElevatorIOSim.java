package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.subsystems.SimMechanism;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSimConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorIOSim extends ElevatorIOTalonFX implements SimMechanism {
  TalonFXSimState motor1Sim = elevatorMotor1.getSimState();
  TalonFXSimState motor2Sim = elevatorMotor2.getSimState();
  ElevatorSim physicsSim =
      new ElevatorSim(
          DCMotor.getFalcon500(2),
          ElevatorSimConstants.GEARING,
          ElevatorSimConstants.CARRIAGE_MASS,
          ElevatorSimConstants.DRUM_RADIUS,
          ElevatorSimConstants.MIN_HEIGHT,
          ElevatorSimConstants.MAX_HEIGHT,
          false,
          ElevatorSimConstants.STARTING_HEIGHT);

  @AutoLogOutput(key = "Elevator/Mechanism")
  LoggedMechanism2d elevatorDrawn =
      new LoggedMechanism2d(Constants.ROBOT_LENGTH, ElevatorSimConstants.MAX_HEIGHT);

  LoggedMechanismRoot2d root = elevatorDrawn.getRoot("elevator", 0, Constants.FLOOR_TO_MECHANISM);
  LoggedMechanismLigament2d elevator =
      root.append(new LoggedMechanismLigament2d("stage", ElevatorSimConstants.STARTING_HEIGHT, 90));

  public ElevatorIOSim() {
    super();
    motor2Sim.Orientation = ChassisReference.Clockwise_Positive;
    initSimVoltage();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    motor1Sim.setSupplyVoltage(RobotController.getBatteryVoltage());
    motor2Sim.setSupplyVoltage(RobotController.getBatteryVoltage());

    physicsSim.setInputVoltage((motor1Sim.getMotorVoltage() + motor2Sim.getMotorVoltage()) / 2);

    inputs.elevator1Connected = true;
    inputs.elevator1Position = physicsSim.getPositionMeters();
    inputs.elevator1Velocity = physicsSim.getVelocityMetersPerSecond();
    inputs.elevator1AppliedVolts = motor1Sim.getMotorVoltage();
    inputs.elevator1CurrentAmps = motor2Sim.getSupplyCurrent();

    inputs.elevator2Connected = true;
    inputs.elevator2Position = physicsSim.getPositionMeters();
    inputs.elevator2Velocity = physicsSim.getVelocityMetersPerSecond();
    inputs.elevator2AppliedVolts = motor2Sim.getMotorVoltage();
    inputs.elevator2CurrentAmps = motor2Sim.getSupplyCurrent();

    physicsSim.update(0.02);

    motor1Sim.setRawRotorPosition(
        physicsSim.getPositionMeters() / ElevatorSimConstants.METERS_PER_ROTATION);
    motor2Sim.setRawRotorPosition(
        physicsSim.getPositionMeters() / ElevatorSimConstants.METERS_PER_ROTATION);
    motor1Sim.setRotorVelocity(
        physicsSim.getVelocityMetersPerSecond() / ElevatorSimConstants.METERS_PER_ROTATION);
    motor2Sim.setRotorVelocity(
        physicsSim.getVelocityMetersPerSecond() / ElevatorSimConstants.METERS_PER_ROTATION);

    elevator.setLength(physicsSim.getPositionMeters());
  }

  @Override
  public double[] getCurrents() {
    return new double[] {physicsSim.getCurrentDrawAmps()};
  }

  public LoggedMechanismLigament2d getLigamentEnd() {
    return elevator;
  }
}

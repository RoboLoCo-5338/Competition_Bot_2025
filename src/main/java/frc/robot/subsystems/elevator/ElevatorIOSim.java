package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.SimMechanism;

public class ElevatorIOSim extends SimMechanism implements ElevatorIO {
  TalonFXSimState motor1Sim = elevatorMotor1.getSimState();
  TalonFXSimState motor2Sim = elevatorMotor2.getSimState();
  ElevatorSim physicsSim =
      new ElevatorSim(
          DCMotor.getFalcon500(2),
          ElevatorConstants.GEARING,
          ElevatorConstants.CARRIAGE_MASS,
          ElevatorConstants.DRUM_RADIUS,
          ElevatorConstants.MIN_HEIGHT,
          ElevatorConstants.MAX_HEIGHT,
          false,
          ElevatorConstants.STARTING_HEIGHT);

  @AutoLogOutput(key = "Elevator/Mechanism")
  LoggedMechanism2d elevatorDrawn =
      new LoggedMechanism2d(Constants.ROBOT_LENGTH, ElevatorConstants.MAX_HEIGHT);

  LoggedMechanismRoot2d root = elevatorDrawn.getRoot("elevator", 0, 0);
  LoggedMechanismLigament2d elevator =
      root.append(new LoggedMechanismLigament2d("stage", ElevatorConstants.STARTING_HEIGHT, 90));

  // this is empty, i will work on it later
  // not if I work on it first
  public ElevatorIOSim() {
    super();
    elevatorMotor1.getConfigurator().apply(getConfiguration(1));

    // TODO does this need to be inverted? idk bro does it?
    elevatorMotor2.getConfigurator().apply(getConfiguration(2));
    // if it does, uncomment the below line.
    motor2Sim.Orientation = ChassisReference.Clockwise_Positive;
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
        physicsSim.getPositionMeters()
            / ElevatorConstants.METERS_PER_ROTATION);
    motor2Sim.setRawRotorPosition(
        physicsSim.getPositionMeters()
            / ElevatorConstants.METERS_PER_ROTATION);
    motor1Sim.setRotorVelocity(
        physicsSim.getVelocityMetersPerSecond()
            / ElevatorConstants.METERS_PER_ROTATION);
    motor2Sim.setRotorVelocity(
        physicsSim.getVelocityMetersPerSecond()
            / ElevatorConstants.METERS_PER_ROTATION);

    elevator.setLength(physicsSim.getPositionMeters());
  }

  @Override
  public void setElevatorVelocity(double velocity) {
    elevatorMotor1.setControl(elevator1VelocityRequest.withVelocity(velocity/ElevatorConstants.METERS_PER_ROTATION));
    elevatorMotor2.setControl(elevator2VelocityRequest.withVelocity(velocity/ElevatorConstants.METERS_PER_ROTATION));
  }

  @Override
  public void setElevatorPosition(double position) {
    elevatorMotor1.setControl(elevator1PositionRequest.withPosition(position/ElevatorConstants.METERS_PER_ROTATION));
    elevatorMotor2.setControl(elevator2PositionRequest.withPosition(position/ElevatorConstants.METERS_PER_ROTATION));
  }

  @Override
  public double[] getCurrents() {
    return new double[] {physicsSim.getCurrentDrawAmps()};
  }

  public LoggedMechanismLigament2d getLigamentEnd() {
    return elevator;
  }
}

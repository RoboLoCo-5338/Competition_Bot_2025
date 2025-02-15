package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  TalonFXSimState motor1Sim = elevatorMotor1.getSimState();
  TalonFXSimState motor2Sim = elevatorMotor2.getSimState();
  ElevatorSim physicsSim = new ElevatorSim(DCMotor.getFalcon500(2), 0, 0, 0, 0, 0, false, 0);
  // this is empty, i will work on it later
  // not if I work on it first
  public ElevatorIOSim() {
    elevatorMotor1.getConfigurator().apply(getConfiguration());

    // TODO does this need to be inverted? idk bro does it?
    elevatorMotor2.getConfigurator().apply(getConfiguration());
    // if it does, uncomment the below line.
    // motor2Sim.Orientation=ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.elevator1Connected = true;
    // inputs.elevator1Position = Units.rotationsToRadians(elevator1Position.getValueAsDouble());
    // inputs.elevator1Velocity = Units.rotationsToRadians(elevator1Velocity.getValueAsDouble());
    // inputs.elevator1AppliedVolts = elevator1AppliedVolts.getValueAsDouble();
    // inputs.elevator1CurrentAmps = elevator1Current.getValueAsDouble();

    inputs.elevator2Connected = true;
    // inputs.elevator2Position = Units.rotationsToRadians(elevator2Position.getValueAsDouble());
    // inputs.elevator2Velocity = Units.rotationsToRadians(elevator2Velocity.getValueAsDouble());
    // inputs.elevator2AppliedVolts = elevator2AppliedVolts.getValueAsDouble();
    // inputs.elevator2CurrentAmps = elevator2Current.getValueAsDouble();
  }

  @Override
  public void setElevatorVelocity(double velocity) {}

  @Override
  public void setElevatorPosition(double position) {}
}

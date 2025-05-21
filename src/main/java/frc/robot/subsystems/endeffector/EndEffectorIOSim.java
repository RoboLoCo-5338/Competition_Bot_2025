package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SimMechanism;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EndEffectorSimConstants;
import java.util.function.Supplier;
import org.dyn4j.geometry.Triangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

public class EndEffectorIOSim implements SimMechanism, EndEffectorIO {
  TalonFXSimState simMotor = endEffectorMotor.getSimState();
  FlywheelSim physicsSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1), EndEffectorSimConstants.MOI, EndEffectorConstants.GEARING),
          DCMotor.getKrakenX60(1));
  IntakeSimulation intakeSim;
  CoralState coralState = CoralState.EFFECTOR;
  Supplier<Pose3d> coralPoseSupplier;
  Supplier<Pose2d> robotPoseSupplier;

  public EndEffectorIOSim(SwerveDriveSimulation driveSim, Supplier<Pose3d> coralPoseSupplier) {
    initSimVoltage();
    endEffectorMotor.getConfigurator().apply(getEndEffectorConfiguration());
    // this.intakeSim =
    //     IntakeSimulation.InTheFrameIntake("Coral", driveSim, Inches.of(34), IntakeSide.BACK, 1);
    this.intakeSim =
        new IntakeSimulation(
            "Coral",
            driveSim,
            new Triangle(
                new Vector2(),
                new Vector2(Units.inchesToMeters(-13.758452), Units.inchesToMeters(13.296619)),
                new Vector2(Units.inchesToMeters(-13.512198), Units.inchesToMeters(-7))),
            1); // TODO: check with mech on this, think its correct
    intakeSim.startIntake(); // the intake sim is started because funnel, not end effector
    new Trigger(() -> intakeSim.getGamePiecesAmount() > 0)
        .onTrue(new InstantCommand(() -> coralState = CoralState.FUNNEL));
    new Trigger(() -> coralState == CoralState.FUNNEL)
        .debounce(0.2) // Waits for a bit simulate coral falling into the
        .and(() -> Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec()) > 25)
        .onTrue(
            new InstantCommand(
                () -> {
                  coralState = CoralState.EFFECTOR;
                }));
    new Trigger(
            () -> Math.abs(Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec())) > 25)
        .debounce(0.2)
        .and(() -> coralState == CoralState.EFFECTOR)
        .onTrue(
            new InstantCommand(
                () -> {
                  coralState = CoralState.EMPTY;
                  intakeSim.obtainGamePieceFromIntake();
                  SimulatedArena.getInstance()
                      .addGamePieceProjectile(
                          new ReefscapeCoralOnFly(
                              driveSim.getSimulatedDriveTrainPose().getTranslation(),
                              coralPoseSupplier.get().getTranslation().toTranslation2d(),
                              driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                              driveSim.getGyroSimulation().getGyroReading(),
                              Meters.of(coralPoseSupplier.get().getZ()),
                              MetersPerSecond.of(5 * Math.signum(getEndEffectorVelocity())),
                              coralPoseSupplier.get().getRotation().getMeasureY()));
                }));
    intakeSim.addGamePieceToIntake();
    new Trigger(DriverStation::isEnabled)
        .onTrue(
            new InstantCommand(
                () -> {
                  intakeSim.startIntake();
                  intakeSim.setGamePiecesCount(1);
                }));
    this.coralPoseSupplier = coralPoseSupplier;
    this.robotPoseSupplier = driveSim::getSimulatedDriveTrainPose;
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    physicsSim.setInputVoltage(simMotor.getMotorVoltage());

    inputs.endEffectorConnected = true;
    inputs.endEffectorVelocity = Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec());
    inputs.endEffectorAppliedVolts = physicsSim.getInputVoltage();
    inputs.endEffectorCurrentAmps = physicsSim.getCurrentDrawAmps();

    physicsSim.update(0.02);

    simMotor.addRotorPosition(
        Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec())
            * 0.02
            * EndEffectorConstants.GEARING);
    simMotor.setRotorVelocity(
        Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec())
            * EndEffectorConstants.GEARING);
    // May move this out and into EndEffector when we have lasercan working

    Logger.recordOutput(
        "Odometry/IntakedCoral",
        (coralState == CoralState.EFFECTOR)
            ? new Pose3d[] {
              new Pose3d(robotPoseSupplier.get())
                  .plus(new Transform3d(new Pose3d(), coralPoseSupplier.get()))
            }
            : new Pose3d[0]);
  }

  @Override
  public void setEndEffectorVelocity(double velocity) {
    endEffectorMotor.setControl(
        endEffectorVelocityRequest.withVelocity(velocity * EndEffectorConstants.GEARING));
  }

  @Override
  public void setEndEffectorSpeed(double speed) {
    endEffectorMotor.set(speed * EndEffectorConstants.GEARING);
  }

  @Override
  public double[] getCurrents() {
    return new double[] {physicsSim.getCurrentDrawAmps()};
  }

  @Override
  public void endEffectorOpenLoop(Voltage voltage) {
    endEffectorMotor.setVoltage(voltage.magnitude());
  }
  @Override
  public double getEndEffectorVelocity() {
    return physicsSim.getAngularVelocityRadPerSec();
  }

  @Override
  public int getLaserCanMeasurement1() {
    return (coralState == CoralState.EFFECTOR) ? 0 : 230;
  }

  @Override
  public int getLaserCanMeasurement2() {
    return (coralState == CoralState.EFFECTOR) ? 0 : 230;
  }

  enum CoralState {
    FUNNEL,
    EFFECTOR,
    EMPTY
  }
}

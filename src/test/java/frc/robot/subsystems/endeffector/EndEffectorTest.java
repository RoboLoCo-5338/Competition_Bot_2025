// package frc.robot.subsystems.endeffector;

// import static edu.wpi.first.units.Units.Seconds;
// import static org.junit.jupiter.api.Assertions.assertTrue;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.simulation.DriverStationSim;
// import edu.wpi.first.wpilibj2.command.RepeatCommand;
// import frc.robot.UnitTestingUtil;
// import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
// import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
// import org.junit.jupiter.api.AfterEach;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;
// import org.junit.jupiter.api.parallel.ResourceAccessMode;
// import org.junit.jupiter.api.parallel.ResourceLock;

// @ResourceLock(value = "SimState", mode = ResourceAccessMode.READ_WRITE)
// public class EndEffectorTest {
//   EndEffector endEffector;
//   boolean stowed = false;
//   boolean canIntakeAlgae = false;

//   @BeforeEach
//   void setUp() {
//     endEffector =
//         new EndEffector(
//             new EndEffectorIOSim(
//                 new SwerveDriveSimulation(DriveTrainSimulationConfig.Default(), new Pose2d()),
//                 () -> new Pose3d(),
//                 () -> stowed,
//                 () -> canIntakeAlgae));
//     UnitTestingUtil.setupTests();
//     Timer.delay(1);
//   }

//   @AfterEach
//   void shutDown() throws Exception {
//     UnitTestingUtil.reset(endEffector);
//   }

//   @Test
//   public void robotIsEnabled() {
//     /* verify that the robot is enabled */
//     assertTrue(DriverStation.isEnabled());
//   }

//   @Test
//   void velocityTest() {
//     System.out.println("Starting Velocity Test");
//     System.out.println("Checking Intake");
//     new RepeatCommand(endEffector.setEndEffectorVelocity(100)).schedule();
//     UnitTestingUtil.fastForward(Seconds.of(10));
//     assert Math.abs(endEffector.getEndEffectorVelocity() - 100) < 1
//         : "End Effector did not reach intake velocity";
//     System.out.println("Checking Outtake");
//     new RepeatCommand(endEffector.setEndEffectorVelocity(-100)).schedule();
//     UnitTestingUtil.fastForward(Seconds.of(10));
//     assert Math.abs(endEffector.getEndEffectorVelocity() + 100) < 1
//         : "End Effector did not reach outtake velocity";
//   }
// }

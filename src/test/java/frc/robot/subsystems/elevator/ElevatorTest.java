// package frc.robot.subsystems.elevator;

// import static edu.wpi.first.units.Units.Seconds;

// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.UnitTestingUtil;
// import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPresetConstants;
// import org.junit.jupiter.api.AfterEach;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;

// public class ElevatorTest {
//   Elevator elevator;

//   @BeforeEach
//   void setUp() {
//     elevator = new Elevator(new ElevatorIOSim());
//     UnitTestingUtil.setupTests();
//     Timer.delay(1);
//   }

//   @AfterEach
//   void shutDown() throws Exception {
//     UnitTestingUtil.reset(elevator);
//   }

//   @Test
//   void positionTest() {
//     System.out.println("Starting Position Test");
//     System.out.println("Checking Stow");
//     UnitTestingUtil.runToCompletion(
//         elevator.setElevatorPosition(ElevatorPresetConstants.ELEVATOR_STOW, 2));
//     assert Math.abs(elevator.input.position - ElevatorPresetConstants.ELEVATOR_STOW)
//             < ElevatorConstants.POSITION_TOLERANCE
//         : "Elevator did not reach stow position";
//     System.out.println("Checking L2");
//     UnitTestingUtil.runToCompletion(
//         elevator.setElevatorPosition(ElevatorPresetConstants.ELEVATOR_L2, 0));
//     assert Math.abs(elevator.input.position - ElevatorPresetConstants.ELEVATOR_L2)
//             < ElevatorConstants.POSITION_TOLERANCE
//         : "Elevator did not reach L2 position";
//     System.out.println("Checking L3");
//     UnitTestingUtil.runToCompletion(
//         elevator.setElevatorPosition(ElevatorPresetConstants.ELEVATOR_L3, 0));
//     assert Math.abs(elevator.input.position - ElevatorPresetConstants.ELEVATOR_L3)
//             < ElevatorConstants.POSITION_TOLERANCE
//         : "Elevator did not reach L3 position";
//     System.out.println("Checking L4");
//     UnitTestingUtil.runToCompletion(
//         elevator.setElevatorPosition(ElevatorPresetConstants.ELEVATOR_L4, 0));
//     assert Math.abs(elevator.input.position - ElevatorPresetConstants.ELEVATOR_L4)
//             < ElevatorConstants.POSITION_TOLERANCE
//         : "Elevator did not reach L4 position";
//     System.out.println("Checking Algae L2");
//     UnitTestingUtil.runToCompletion(
//         elevator.setElevatorPosition(ElevatorPresetConstants.ELEVATOR_L2_ALGAE, 0));
//     assert Math.abs(elevator.input.position - ElevatorPresetConstants.ELEVATOR_L2_ALGAE)
//             < ElevatorConstants.POSITION_TOLERANCE
//         : "Elevator did not reach Algae L2 position";
//     System.out.println("Checking Algae L3");
//     UnitTestingUtil.runToCompletion(
//         elevator.setElevatorPosition(ElevatorPresetConstants.ELEVATOR_L3_ALGAE, 0));
//     assert Math.abs(elevator.input.position - ElevatorPresetConstants.ELEVATOR_L3_ALGAE)
//             < ElevatorConstants.POSITION_TOLERANCE
//         : "Elevator did not reach Algae L3 position";
//     System.out.println("Position Test Completed Successfully");
//   }

//   @Test
//   void velocityTest() {
//     System.out.println("Starting Velocity Test");
//     System.out.println("Checking Elevator Velocity");
//     elevator.setElevatorVelocity(() -> 25);
//     UnitTestingUtil.fastForward(Seconds.of(1));
//     assert Math.abs(elevator.input.elevator1Velocity - 25) < 1
//         : "Elevator did not reach velocity of 100";
//     System.out.println("Checking Elevator Reverse Velocity");
//     elevator.setElevatorVelocity(() -> -25);
//     UnitTestingUtil.fastForward(Seconds.of(1));
//     assert Math.abs(elevator.input.elevator1Velocity + 25) < 1
//         : "Elevator did not reach reverse velocity of -100";
//     System.out.println("Checking Elevator Stop Velocity");
//     elevator.setElevatorVelocity(() -> 0);
//     UnitTestingUtil.fastForward(Seconds.of(1));
//     assert Math.abs(elevator.input.elevator1Velocity) < 0.001
//         : "Elevator did not stop at velocity of 0";
//     System.out.println("Velocity Test Completed Successfully");
//   }
// }

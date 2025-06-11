// package frc.robot.subsystems.arm;

// import frc.robot.UnitTestingUtil;
// import frc.robot.subsystems.arm.ArmConstants.ArmPresetConstants;
// import org.junit.jupiter.api.AfterEach;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;
// import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

// public class ArmTest {
//   Arm arm;

//   @BeforeEach
//   void setUp() {
//     UnitTestingUtil.setupTests();
//     arm = new Arm(new ArmIOSim(new LoggedMechanismLigament2d("test", 0, 0)));
//   }

//   @AfterEach
//   void shutDown() throws Exception {
//     UnitTestingUtil.reset(arm);
//   }

//   @Test
//   // TODO: Implement velocity test after fixing velocity overshoot
//   void velocityTest() {}

//   @Test
//   void positionTest() {
//     // TODO: maybe implement holding tests once holding becomes possible
//     System.out.println("Starting Position Test");
//     System.out.println("Checking Stow Initial");
//     UnitTestingUtil.runToCompletion(arm.setArmPosition(ArmPresetConstants.ARM_STOW_INITIAL));
//     assert Math.abs(arm.input.position - ArmPresetConstants.ARM_STOW_INITIAL)
//             < ArmConstants.POSITION_TOLERANCE
//         : "Arm did not reach stow initial position";
//     System.out.println("Checking Stow Final");
//     UnitTestingUtil.runToCompletion(arm.setArmPosition(ArmPresetConstants.ARM_STOW_FINAL));
//     assert Math.abs(arm.input.position - ArmPresetConstants.ARM_STOW_FINAL)
//             < ArmConstants.POSITION_TOLERANCE
//         : "Arm did not reach stow final position";
//     System.out.println("Checking L2 L3");
//     UnitTestingUtil.runToCompletion(arm.setArmPosition(ArmPresetConstants.ARM_L2_L3));
//     assert Math.abs(arm.input.position - ArmPresetConstants.ARM_L2_L3)
//             < ArmConstants.POSITION_TOLERANCE
//         : "Arm did not reach L2 L3 position";
//     System.out.println("Checking L4");
//     UnitTestingUtil.runToCompletion(arm.setArmPosition(ArmPresetConstants.ARM_L4));
//     assert Math.abs(arm.input.position - ArmPresetConstants.ARM_L4)
//             < ArmConstants.POSITION_TOLERANCE
//         : "Arm did not reach L4 position";
//     System.out.println("Checking NET");
//     UnitTestingUtil.runToCompletion(arm.setArmPosition(ArmPresetConstants.ARM_NET));
//     assert Math.abs(arm.input.position - ArmPresetConstants.ARM_NET)
//             < ArmConstants.POSITION_TOLERANCE
//         : "Arm did not reach NET position";
//     System.out.println("Checking Algae");
//     UnitTestingUtil.runToCompletion(arm.setArmPosition(ArmPresetConstants.ARM_ALGAE));
//     assert Math.abs(arm.input.position - ArmPresetConstants.ARM_ALGAE)
//             < ArmConstants.POSITION_TOLERANCE
//         : "Arm did not reach Algae position";
//     System.out.println("Position test completed successfully");
//   }
// }

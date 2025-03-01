package frc.robot;

import static java.util.Map.entry;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.led.LED;
import java.util.Map;

public class ButtonBindings {
  private Drive drive;
  private LED led;
  private Elevator elevator;
  private GroundIntake groundIntake;
  private EndEffector endEffector;
  private Arm arm;
  private Climb climb;

  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private double exponentialVariable = 25.0;

  // maps button names and function names to their respective objects
  public Map<String, Trigger> buttonMappings;
  public Map<String, Command> functionBindings;

  // these hashmaps are what happens when the button is pressed
  public Map<String, String> buttonToFunction;

  // these hashmaps are what happens when the button is not pressed
  public Map<String, String> negatedButtonToFunction;

  public ButtonBindings(
      Drive drive,
      LED led,
      Elevator elevator,
      GroundIntake groundIntake,
      EndEffector endEffector,
      Climb climb,
      Arm arm) {
    this.drive = drive;
    ActionBindings.drive = drive;
    ActionBindings.arm = arm;
    ActionBindings.led = led;
    ActionBindings.elevator = elevator;
    ActionBindings.groundIntake = groundIntake;
    ActionBindings.endEffector = endEffector;
    ActionBindings.climb = climb;
    ActionBindings.arm = arm;

    setUpFunctionBindings();
    setUpButtonMappings();
    setUpButtonBindings();
    connectButtonToFunction();
    defaultButtonBindings(); // handles any input coming from joysticks
  }

  @SuppressWarnings({"rawtypes", "unchecked"})
  private void setUpFunctionBindings() {

    functionBindings =
        Map.ofEntries(
            // entry("Lock to Zero", ActionBindings.lockToZero()),
            // entry("Climb Preset", ActionBindings.climbPreset()),
            // entry("Gyro Reset", ActionBindings.gyroReset()),
            // entry("Manual Climb Down", ActionBindings.manualClimbDown()),
            // entry("Manual Climb Up", ActionBindings.manualClimbUp()),
            // entry("L3 Preset", ActionBindings.l3Preset()),
            // entry("Elevator Slow", ActionBindings.elevatorSlow()),
            // entry("L2 Preset", ActionBindings.l2Preset()),
            // entry("L4 Preset", ActionBindings.l4Preset()),
            // entry("Elevator Fast", ActionBindings.elevatorFast()),
            // entry("Manual Arm Down", ActionBindings.manualArmDown()),
            // entry("Manual Arm Up", ActionBindings.manualArmUp()),
            // // entry("Net Preset", ActionBindings.netPreset()),
            // entry("Ground Intake In", ActionBindings.groundIntakeIn()),
            // entry("Ground Intake Out", ActionBindings.groundIntakeOut())
            // entry("End Effector In", ActionBindings.endEffectorIn()),
            // entry("End Effector Out", ActionBindings.endEffectorOut()),
            // entry("Ground Intake Slow", ActionBindings.groundIntakeSlow()),
            // entry("Ground Intake Fast", ActionBindings.groundIntakeFast()),
            // entry("Elevator Up", ActionBindings.manualElevatorUp()),
            // entry("Elevator Down", ActionBindings.manualElevatorDown()),
            // entry("Arm Up", ActionBindings.manualArmUp()),
            // entry("Arm Down", ActionBindings.manualArmDown()));
            );
  }

  @SuppressWarnings("unchecked")
  private void setUpButtonMappings() {
    buttonMappings =
        Map.ofEntries(
            // entry("D - x", driveController.x()),
            // entry("D - a", driveController.a()),
            // entry("D - y", driveController.y()),
            // entry("D - b", driveController.b()),
            // entry("D - Right Joystick Button", driveController.rightStick()),
            // entry("D - Left Joystick Button", driveController.leftStick()),
            // entry("D - Left Bumper", driveController.leftBumper()),
            // entry("D - Left Trigger", driveController.leftTrigger()),
            // entry("D - Right Bumper", driveController.rightBumper()),
            // entry("D - D-Pad Up", driveController.povUp()),
            // entry("D - D-Pad Down", driveController.povDown()),
            // entry("D - D-Pad Left", driveController.povLeft()),
            // entry("D - D-Pad Right", driveController.povRight()),
            // entry("D - Right Trigger", new Trigger(() -> driveController.getRawAxis(5) > -0.5)),
            // entry("O - a", operatorController.a()),
            // entry("O - y", operatorController.y()),
            // entry("O - B", operatorController.b()),
            // entry("O - Right Bumper", operatorController.rightBumper()),
            // entry("O - Left Bumper", operatorController.leftBumper()),
            // entry("O - Right Trigger", new Trigger(() -> operatorController.getRawAxis(5) >
            // -0.5)),
            // entry("O - Left Trigger", operatorController.leftTrigger()),
            // entry("O - D-Pad Down", operatorController.povDown()),
            // entry("O - D-Pad Up", operatorController.povUp()),
            // entry("O - D-Pad Left", operatorController.povLeft()));
            );
  }

  @SuppressWarnings({"rawtypes", "unchecked"})
  private void setUpButtonBindings() {
    // Driver Controller Bindings
    buttonToFunction =
        Map.ofEntries(
            // entry("D - x", "Lock to Zero"),
            // entry("D - a", "Climb Preset"),
            // entry("D - y", "Gyro Reset"),
            // entry("D - Right Joystick Button", "Manual Climb Down"),
            // entry("D - Left Joystick Button", "Manual Climb Up"),
            // entry("D - Left Bumper", "L3 Preset"),
            // entry("D - Left Trigger", "Elevator Slow"),
            // entry("D - Right Bumper", "L2 Preset"),
            // entry("D - Right Trigger", "L4 Preset"),
            // entry("D - D-Pad Up", "Elevator Up"),
            // entry("D - D-Pad Down", "Elevator Down"),
            // entry("D - D-Pad Left", "Arm Up"),
            // entry("D - D-Pad Right", "Arm Down"),
            // entry("O - a", "Manual Arm Down"),
            // entry("O - y", "Manual Arm Up"),
            // entry("O - b", "Net Preset"),
            //  entry("O - Right Bumper", "Ground Intake In"),
            //  entry("O - Left Bumper", "Ground Intake Out"),
            // entry("O - Right Trigger", "End Effector In"),
            // entry("O - Left Trigger", "End Effector Out"),
            // entry("O - D-Pad Down", "Ground Intake In"),
            // entry("O - D-Pad Up", "Ground Intake Slow"),
            // entry("O - D-Pad Left", "Ground Intake Out"));
            );

    // Negated Operator Controller Bindings
    negatedButtonToFunction =
        Map.ofEntries(
            entry("O - D-Pad Up", "Ground Intake Fast"),
            entry("D - Left Trigger", "Elevator Fast"));
  }

  public static void periodic() {}

  private void connectButtonToFunction() {
    for (String button : buttonToFunction.keySet()) {
      if (buttonMappings.get(button) == null) {
        System.out.println("Button " + button + " not found");
        continue;
      }
      if (functionBindings.get(buttonToFunction.get(button)) == null) {
        System.out.println(buttonToFunction.get(button));
        System.out.println(functionBindings.get(buttonToFunction.get(button)));
        System.out.println(functionBindings.keySet());
        System.out.println("Function " + buttonToFunction.get(button) + " not found");
        continue;
      }

      buttonMappings
          .get(button)
          .onTrue(ButtonBindings.debugCommand(button, buttonToFunction.get(button)))
          .whileTrue(functionBindings.get(buttonToFunction.get(button)));
    }
    for (String button : negatedButtonToFunction.keySet()) {

      if (buttonMappings.get(button) == null) {
        System.out.println("Negated Button " + button + " not found");
        continue;
      }
      if (functionBindings.get(negatedButtonToFunction.get(button)) == null) {
        System.out.println(
            "Negated Function " + negatedButtonToFunction.get(button) + " not found");
        continue;
      }

      buttonMappings
          .get(button)
          .whileFalse(functionBindings.get(negatedButtonToFunction.get(button)));
    }
  }

  public void defaultButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    // handles translation and rotation driving (i think!?)
  }

  // public double joystickExponentialFunction(double x) {
  //   return (1.0 / (exponentialVariable - 1)) * (Math.pow(exponentialVariable, x) - 1);
  // }

  public static Command debugCommand(String button, String action) {

    return new InstantCommand(
        () -> {
          NetworkTableInstance.getDefault()
              .getStringTopic("Last Button Pressed")
              .publish()
              .set(button);
          NetworkTableInstance.getDefault()
              .getStringTopic("Last Action Done")
              .publish()
              .set(action);
        });
  }

  public static Command blankCommand(String name) {
    return new InstantCommand(
        () -> {
          System.out.println(name + " is not implemented!");
        });
  }
}

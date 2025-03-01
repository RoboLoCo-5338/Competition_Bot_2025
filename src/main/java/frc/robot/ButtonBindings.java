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

  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final ActionBindings actionBindings;
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
    this.actionBindings =
        new ActionBindings(
            drive,
            driveController,
            operatorController,
            climb,
            elevator,
            groundIntake,
            endEffector,
            led,
            arm);

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
            entry("Lock to Zero", this.actionBindings.lockToZero()),
            entry("Climb Preset", this.actionBindings.climbPreset()),
            entry("Gyro Reset", this.actionBindings.gyroReset()),
            entry("Manual Climb Down", this.actionBindings.manualClimbDown()),
            entry("Manual Climb Up", this.actionBindings.manualClimbUp()),
            entry("L3 Preset", this.actionBindings.l3Preset()),
            entry("Elevator Slow", this.actionBindings.elevatorSlow()),
            entry("L2 Preset", this.actionBindings.l2Preset()),
            entry("L4 Preset", this.actionBindings.l4Preset()),
            entry("Elevator Fast", this.actionBindings.elevatorFast()),
            entry("Manual Arm Down", this.actionBindings.manualArmDown()),
            entry("Manual Arm Up", this.actionBindings.manualArmUp()),
            entry("Net Preset", this.actionBindings.netPreset()),
            entry("Ground Intake In", this.actionBindings.groundIntakeIn()),
            entry("Ground Intake Out", this.actionBindings.groundIntakeOut()),
            entry("End Effector In", this.actionBindings.endEffectorIn()),
            entry("End Effector Out", this.actionBindings.endEffectorOut()),
            entry("Ground Intake Slow", this.actionBindings.groundIntakeSlow()),
            entry("Ground Intake Fast", this.actionBindings.groundIntakeFast()),
            entry("Elevator Up", this.actionBindings.manualElevatorUp()),
            entry("Elevator Down", this.actionBindings.manualElevatorDown()),
            entry("Arm Up", this.actionBindings.manualArmUp()),
            entry("Arm Down", this.actionBindings.manualArmDown()),
            entry("Elevator Stop", this.actionBindings.manualElevatorStop()),
            entry("Stop Climb", this.actionBindings.stopClimb()),
            entry("End Effector Stop", this.actionBindings.endEffectorStop()),
            entry("Ground Intake Stop", this.actionBindings.groundIntakeStop()),
            entry("Arm Stop", this.actionBindings.manualArmStop()));

    // stopping all the things

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
            entry("D - Left Trigger", "Elevator Fast"),
            entry("D - D-Pad Right", "Arm Stop"),
            entry("D - D-Pad Left", "Arm Stop"),
            entry("D - D-Pad Down", "Elevator Stop"),
            entry("D - D-Pad Up", "Elevator Stop"),
            entry("O - D-Pad Down", "Ground Intake Stop"),
            entry("O - D-Pad Left", "Ground Intake Stop"),
            entry("O - Right Trigger", "End Effector Stop"),
            entry("O - Left Trigger", "End Effector Stop"),
            entry("D - Right Joystick Button", "Stop Climb"),
            entry("D - Left Joystick Button", "Stop Climb"));
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
      System.out.println(
          "Negated Button " + button + " connecting to " + negatedButtonToFunction.get(button));
      if (buttonMappings.get(button) == null) {
        System.out.println("Negated Button " + button + " not found");
        continue;
      }
      if (functionBindings.get(negatedButtonToFunction.get(button)) == null) {
        System.out.println(
            "Negated Function " + negatedButtonToFunction.get(button) + " not found");
        continue;
      }
      System.out.println(
          "Negated Button " + button + " connected to " + negatedButtonToFunction.get(button));
      buttonMappings
          .get(button)
          .onFalse(ButtonBindings.debugCommand(button, negatedButtonToFunction.get(button)))
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
    // elevator.setDefaultCommand(
    //   elevator.setElevatorVelocity(operatorController.getLeftY()/10)
    // );
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

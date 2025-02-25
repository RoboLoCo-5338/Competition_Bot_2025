package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.EndEffectorCommands;
import frc.robot.commands.GroundIntakeCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.led.LED;
import java.util.HashMap;

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
  public HashMap<String, Trigger> buttonMappings = new HashMap<>();
  public HashMap<String, Command> functionBindings = new HashMap<>();

  // these hashmaps are what happens when the button is pressed
  public HashMap<String, String> buttonToFunction = new HashMap<>();

  // these hashmaps are what happens when the button is not pressed
  public HashMap<String, String> negatedButtonToFunction = new HashMap<>();

  public ButtonBindings(
      Drive drive,
      LED led,
      Elevator elevator,
      GroundIntake groundIntake,
      EndEffector endEffector,
      Climb climb,
      Arm arm) {
    this.drive = drive;
    this.led = led;
    this.elevator = elevator;
    this.groundIntake = groundIntake;
    this.endEffector = endEffector;
    this.climb = climb;
    this.arm = arm;

    setUpFunctionBindings();
    setUpButtonMappings();
    setUpButtonBindings();
    connectButtonToFunction();
    defaultButtonBindings(); // handles any input coming from joysticks
  }

  private void setUpFunctionBindings() {
    functionBindings.put("Lock to Zero", this.lockToZero());
    functionBindings.put("Climb Preset", this.climbPreset());
    functionBindings.put("Gyro Reset", this.gyroReset());
    functionBindings.put("Manual Climb Down", this.manualClimbDown());
    functionBindings.put("Manual Climb Up", this.manualClimbUp());
    functionBindings.put("L3 Preset", this.l3Preset());
    functionBindings.put("Elevator Slow", this.elevatorSlow());
    functionBindings.put("L2 Preset", this.l2Preset());
    functionBindings.put("L4 Preset", this.l4Preset());
    functionBindings.put("Elevator Fast", this.elevatorFast());
    functionBindings.put("Manual Arm Down", this.manualArmDown());
    functionBindings.put("Manual Arm Up", this.manualArmUp());
    functionBindings.put("Net Preset", this.netPreset());
    functionBindings.put("Ground Intake In", this.groundIntakeIn());
    functionBindings.put("Ground Intake Out", this.groundIntakeOut());
    functionBindings.put("End Effector In", this.endEffectorIn());
    functionBindings.put("End Effector Out", this.endEffectorOut());
    functionBindings.put("Ground Intake Slow", this.groundIntakeSlow());
    functionBindings.put("Ground Intake Fast", this.groundIntakeFast());
    functionBindings.put("Elevator Up", this.manualElevatorUp());
    functionBindings.put("Elevator Down", this.manualElevatorDown());
  }

  private void setUpButtonMappings() {
    buttonMappings.put("D - x", driveController.x());
    buttonMappings.put("D - a", driveController.a());
    buttonMappings.put("D - y", driveController.y());
    buttonMappings.put("D - b", driveController.b());
    buttonMappings.put("D - Right Joystick Button", driveController.rightStick());
    buttonMappings.put("D - Left Joystick Button", driveController.leftStick());
    buttonMappings.put("D - Left Bumper", driveController.leftBumper());
    buttonMappings.put("D - Left Trigger", driveController.leftTrigger());
    buttonMappings.put("D - Right Bumper", driveController.rightBumper());
    buttonMappings.put("D - D-Pad Up", driveController.povUp());
    buttonMappings.put("D - D-Pad Down", driveController.povDown());
    buttonMappings.put("D - D-Pad Left", driveController.povLeft());

    buttonMappings.put(
        "D - Right Trigger",
        new Trigger(
            () ->
                driveController.getRawAxis(5)
                    > -0.5)); // the .rightTrigger() trigger does not work for me - maybe its a
    // linux thing - or WPILib is slightly broken

    buttonMappings.put("O - a", operatorController.a());
    buttonMappings.put("O - y", operatorController.y());
    buttonMappings.put("O - B", operatorController.b());
    buttonMappings.put("O - Right Bumper", operatorController.rightBumper());
    buttonMappings.put("O - Left Bumper", operatorController.leftBumper());
    buttonMappings.put(
        "O - Right Trigger", new Trigger(() -> operatorController.getRawAxis(5) > -0.5));
    buttonMappings.put("O - Left Trigger", operatorController.leftTrigger());
    buttonMappings.put("O - D-Pad Down", operatorController.povDown());
    buttonMappings.put("O - D-Pad Up", operatorController.povUp());
    buttonMappings.put("O - D-Pad Left", operatorController.povLeft());
  }

  private void setUpButtonBindings() {
    // Driver Controller Bindings
    buttonToFunction.put("D - x", "Lock to Zero");
    buttonToFunction.put("D - a", "Climb Preset");
    buttonToFunction.put("D - y", "Gyro Reset");
    buttonToFunction.put("D - Right Joystick Button", "Manual Climb Down");
    buttonToFunction.put("D - Left Joystick Button", "Manual Climb Up");
    buttonToFunction.put("D - Left Bumper", "L3 Preset");
    buttonToFunction.put("D - Left Trigger", "Elevator Slow");
    buttonToFunction.put("D - Right Bumper", "L2 Preset");
    buttonToFunction.put("D - Right Trigger", "L4 Preset");
    buttonToFunction.put("D - D-Pad Up", "Elevator Up");
    buttonToFunction.put("D - D-Pad Down", "Elevator Down");
    // Negated Driver Controller Bindings
    negatedButtonToFunction.put("D - Left Trigger", "Elevator Fast");

    // Operator Controller Bindings
    buttonToFunction.put("O - a", "Manual Arm Down");
    buttonToFunction.put("O - y", "Manual Arm Up");
    buttonToFunction.put("O - b", "Net Preset");
    buttonToFunction.put("O - Right Bumper", "Ground Intake In");
    buttonToFunction.put("O - Left Bumper", "Ground Intake Out");
    buttonToFunction.put("O - Right Trigger", "End Effector In");
    buttonToFunction.put("O - Left Trigger", "End Effector Out");
    buttonToFunction.put("O - D-Pad Down", "Ground Intake In");
    buttonToFunction.put("O - D-Pad Up", "Ground Intake Slow");
    buttonToFunction.put("O - D-Pad Left", "Ground Intake Out");

    // Negated Operator Controller Bindings
    negatedButtonToFunction.put("O - D-Pad Up", "Ground Intake Fast");
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
            () -> joystickExponentialFunction(-driveController.getLeftY()),
            () -> -driveController.getLeftX(),
            () ->
                -driveController
                    .getRightX())); // handles translation and rotation driving (i think!?)
  }

  public double joystickExponentialFunction(double x) {
    return (1.0 / (exponentialVariable - 1)) * (Math.pow(exponentialVariable, x) - 1);
  }

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

  public Command lockToZero() {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -driveController.getLeftY(),
        () -> -driveController.getLeftX(),
        () -> new Rotation2d());
  }

  public Command climbPreset() {
    return ButtonBindings.blankCommand("climbPreset");
  }

  public Command gyroReset() {
    return Commands.runOnce(
            () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
            drive)
        .ignoringDisable(true);
  }

  public Command manualClimbDown() {
    return ClimbCommands.moveClimbArm(climb, -0.4);
  }

  public Command manualClimbUp() {
    return ClimbCommands.moveClimbArm(climb, 0.4);
  }

  public Command manualElevatorUp() {
    return ElevatorCommands.moveElevator(elevator, 0.4);
  }

  public Command manualElevatorDown() {
    return ElevatorCommands.moveElevator(elevator, -0.4);
  }

  public Command l3Preset() {
    return ButtonBindings.blankCommand("l3Preset");
  }

  public Command elevatorSlow() {
    return ButtonBindings.blankCommand("elevatorSlow");
  }

  public Command l2Preset() {
    return ButtonBindings.blankCommand("l2Preset");
  }

  public Command l4Preset() {
    return ButtonBindings.blankCommand("l4Preset");
  }

  public Command elevatorFast() {
    return ButtonBindings.blankCommand("elevatorFast");
  }

  public Command manualArmDown() {
    return ArmCommands.moveArm(arm, -0.4);
  }

  public Command manualArmUp() {
    return ArmCommands.moveArm(arm, 0.4);
  }

  public Command netPreset() {
    return ButtonBindings.blankCommand("netPreset");
  }

  public Command groundIntakeIn() {
    return GroundIntakeCommands.moveGroundIntakeArm(groundIntake, -0.4, 0.4);
  }

  public Command groundIntakeOut() {
    return GroundIntakeCommands.moveGroundIntakeArm(groundIntake, 0.4, -0.4);
  }

  public Command endEffectorIn() {
    return EndEffectorCommands.moveEndEffector(endEffector, 0.4);
  }

  public Command endEffectorOut() {
    return EndEffectorCommands.moveEndEffector(endEffector, -0.4);
  }

  public Command groundIntakeSlow() {
    return ButtonBindings.blankCommand("groundIntakeSlow");
  }

  public Command groundIntakeFast() {
    return ButtonBindings.blankCommand("groundIntakeFast");
  }
}

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.led.LED;

public class ButtonBindings {
  private Drive drive;
  private LED led;
  private Elevator elevator;
  private GroundIntake groundIntake;
  private EndEffector endEffector;
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private double exponentialVariable = 25.0;

  // maps button names and function names to their respective objects
  public HashMap<String, Trigger> buttonMappings = new HashMap<>();
  public HashMap<String, Command> functionBindings = new HashMap<>();


  // these hashmaps are what happens when the button is pressed
  public HashMap<String, String> driverButtonToFunction = new HashMap<>();
  public HashMap<String, String> operatorButtonToFunction = new HashMap<>();

  // these hashmaps are what happens when the button is not pressed
  public HashMap<String, String> negatedDriverButtonToFunction = new HashMap<>();
  public HashMap<String, String> negatedOperatorButtonToFunction = new HashMap<>();

  public ButtonBindings(Drive drive, LED led, Elevator elevator, GroundIntake groundIntake, EndEffector endEffector) {
    this.drive = drive;
    this.led = led;
    this.elevator = elevator;
    this.groundIntake = groundIntake;
    this.endEffector = endEffector;
    setUpFunctionBindings();
    setUpButtonMappings();
    setUpButtonBindings();
    connectButtonToFunction();
      }
    
    private void setUpFunctionBindings() {
      functionBindings.put("Lock to Zero", this.lockToZero());
      

    }
    private void setUpButtonMappings() {
      buttonMappings.put("x", driveController.x());
    
    }
    private void setUpButtonBindings() {
      // Driver Controller Bindings
      driverButtonToFunction.put("x", "Lock to Zero");
      driverButtonToFunction.put("a", "Climb Present");
      driverButtonToFunction.put("y", "Gyro Reset");
      driverButtonToFunction.put("Right Joystick Button", "Manual Climb Down");
      driverButtonToFunction.put("Left Joystick Button", "Manual Climb Up");
      driverButtonToFunction.put("Left Bumper", "L3 Preset");
      driverButtonToFunction.put("Left Trigger", "Elevator Slow");
      driverButtonToFunction.put("Right Bumper", "L2 Preset");
      driverButtonToFunction.put("Right Trigger", "L4 Preset");
      // Negated Driver Controller Bindings
      negatedDriverButtonToFunction.put("Left Trigger", "Elevator Fast");

      // Operator Controller Bindings
      operatorButtonToFunction.put("a", "Manual Arm Down");
      operatorButtonToFunction.put("y", "Manual Arm Up");
      operatorButtonToFunction.put("B", "Net Preset");
      operatorButtonToFunction.put("Right Bumper", "Ground Intake In");
      operatorButtonToFunction.put("Left Bumper", "Ground Intake Out");
      operatorButtonToFunction.put("Right Trigger", "End Effector In");
      operatorButtonToFunction.put("Left Trigger", "End Effector Out");
      operatorButtonToFunction.put("D-Pad Down", "Ground Intake In");
      operatorButtonToFunction.put("D-Pad Up", "Ground Intake Slow");
      operatorButtonToFunction.put("D-Pad Left", "Ground Intake Out");
  

      // Negated Operator Controller Bindings
      negatedOperatorButtonToFunction.put("D-Pad Up", "Ground Intake Fast");
  }
  

    private void connectButtonToFunction() {
      buttonMappings.forEach(
          (buttonName, button) -> {
            button.whileTrue(functionBindings.get(buttonName));
          });

      }
      public void defaultButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> joystickExponentialFunction(-driveController.getLeftY()),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));
  }
  public double joystickExponentialFunction(double x) {
      return (1.0 / (exponentialVariable - 1)) * (Math.pow(exponentialVariable, x) - 1);
    }
  public Command lockToZero() {
      return DriveCommands.joystickDriveAtAngle(
              drive,
              () -> -driveController.getLeftY(),
              () -> -driveController.getLeftX(),
              () -> new Rotation2d());
  }
}

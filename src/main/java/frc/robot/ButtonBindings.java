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
  public HashMap<String, String> buttonToFunction = new HashMap<>();
 
  // these hashmaps are what happens when the button is not pressed
  public HashMap<String, String> negatedButtonToFunction = new HashMap<>();
 

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

      

    }
    private void setUpButtonMappings() {
      buttonMappings.put("D - x", driveController.x());
      buttonMappings.put("D - a", driveController.a());
      buttonMappings.put("D - y", driveController.y());
      buttonMappings.put("D - Right Joystick Button", driveController.rightStick());
      buttonMappings.put("D - Left Joystick Button", driveController.leftStick());
      buttonMappings.put("D - Left Bumper", driveController.leftBumper());
      buttonMappings.put("D - Left Trigger", driveController.leftTrigger());
      buttonMappings.put("D - Right Bumper", driveController.rightBumper());
      buttonMappings.put("D - Right Trigger", driveController.rightTrigger());
      
      buttonMappings.put("O - a", operatorController.a());
      buttonMappings.put("O - y", operatorController.y());
      buttonMappings.put("O - B", operatorController.b());
      buttonMappings.put("O - Right Bumper", operatorController.rightBumper());
      buttonMappings.put("O - Left Bumper", operatorController.leftBumper());
      buttonMappings.put("O - Right Trigger", operatorController.rightTrigger());
      buttonMappings.put("O - Left Trigger", operatorController.leftTrigger());
      buttonMappings.put("O - D-Pad Down", operatorController.povDown());
      buttonMappings.put("O - D-Pad Up", operatorController.povUp());
      buttonMappings.put("O - D-Pad Left", operatorController.povLeft());
        


    
    }
    private void setUpButtonBindings() {
      // Driver Controller Bindings
      buttonToFunction.put("x", "Lock to Zero");
      buttonToFunction.put("a", "Climb Preset");
      buttonToFunction.put("y", "Gyro Reset");
      buttonToFunction.put("Right Joystick Button", "Manual Climb Down");
      buttonToFunction.put("Left Joystick Button", "Manual Climb Up");
      buttonToFunction.put("Left Bumper", "L3 Preset");
      buttonToFunction.put("Left Trigger", "Elevator Slow");
      buttonToFunction.put("Right Bumper", "L2 Preset");
      buttonToFunction.put("Right Trigger", "L4 Preset");
      // Negated Driver Controller Bindings
      negatedButtonToFunction.put("Left Trigger", "Elevator Fast");

      // Operator Controller Bindings
      buttonToFunction.put("a", "Manual Arm Down");
      buttonToFunction.put("y", "Manual Arm Up");
      buttonToFunction.put("B", "Net Preset");
      buttonToFunction.put("Right Bumper", "Ground Intake In");
      buttonToFunction.put("Left Bumper", "Ground Intake Out");
      buttonToFunction.put("Right Trigger", "End Effector In");
      buttonToFunction.put("Left Trigger", "End Effector Out");
      buttonToFunction.put("D-Pad Down", "Ground Intake In");
      buttonToFunction.put("D-Pad Up", "Ground Intake Slow");
      buttonToFunction.put("D-Pad Left", "Ground Intake Out");
  

      // Negated Operator Controller Bindings
      negatedButtonToFunction.put("D-Pad Up", "Ground Intake Fast");
  }
  

    private void connectButtonToFunction() {
      for (String button : buttonToFunction.keySet()) {
        buttonMappings.get(button).whileTrue(functionBindings.get(buttonToFunction.get(button)));
      }
      for (String button : negatedButtonToFunction.keySet()) {
        buttonMappings.get(button).whileFalse(functionBindings.get(negatedButtonToFunction.get(button)));
      }
    }


    public void defaultButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> joystickExponentialFunction(-driveController.getLeftY()),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX())); //handles translation and rotation driving (i think!?)
        

    
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
  public Command climbPreset() {
      return null; //TODO Implement Climb Preset
  }
  public Command gyroReset() {
      return DriveCommands.gyroReset(drive); // TODO Implement Gyro Reset
  }
  public Command manualClimbDown() {
      return null; // TODO Implement Manual Climb Down
  }
  public Command manualClimbUp() {
      return null; // TODO Implement Manual Climb Up
  }
  public Command l3Preset() {
      return null; // TODO Implement L3 Preset
  }
  public Command elevatorSlow() {
      return null; // TODO Implement Elevator Slow
  }
  public Command l2Preset() {
      return null; // TODO Implement L2 Preset
  }
  public Command l4Preset() {
      return null; // TODO Implement L4 Preset
  }
  public Command elevatorFast() {
      return null; // TODO Implement Elevator Fast
  }
  public Command manualArmDown() {
      return null; // TODO Implement Manual Arm Down
  }
  public Command manualArmUp() {
      return null; // TODO Implement Manual Arm Up
  }
  public Command netPreset() {
      return null; // TODO Implement Net Preset
  }
  public Command groundIntakeIn() {
      return null; // TODO Implement Ground Intake In
  }
  public Command groundIntakeOut() {
      return null; // TODO Implement Ground Intake Out
  }
  public Command endEffectorIn() {
      return null; // TODO Implement End Effector In
  }
  public Command endEffectorOut() {
      return null; // TODO Implement End Effector Out
  }
  public Command groundIntakeSlow() {
      return null; // TODO Implement Ground Intake Slow
  }
  public Command groundIntakeFast() {
      return null; // TODO Implement Ground Intake Fast
  }
  

}

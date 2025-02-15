package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.EndEffector.EndEffector;

public class ArmCommands {

    public static Command moveArm(Arm arm, EndEffector endEffector,double speed){
        return new ParallelCommandGroup(
            new InstantCommand(()-> arm.setArmVelocity(speed)),
            new InstantCommand(() -> endEffector.setEndEffectorVelocity(speed)));
    }

    
}

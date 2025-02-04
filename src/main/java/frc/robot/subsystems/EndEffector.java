package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
    private SparkMax shootMotor1;
    private RelativeEncoder shootEncoder1;

    public EndEffector() {
        
        shootMotor1 = new SparkMax(Constants.DriveConstants.kShooter1CanId, MotorType.kBrushless);

        shootEncoder1 = shootMotor1.getEncoder();
    
    }

    public void shooterForward() {
        shootMotor1.set(0.3);

    }

    public void shooterReverse() {
        shootMotor1.set(-0.3);
    }
     public void shooterReverseSlow() {
        shootMotor1.set(-0.1);
    }

    public void shooterStop() {
        shootMotor1.set(0);
    }

    public double getEncoderPosition(){
        return (Math.abs(shootEncoder1.getPosition()));
    }
}

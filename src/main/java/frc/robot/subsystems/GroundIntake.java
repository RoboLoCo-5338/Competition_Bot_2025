package frc.robot.subsystems;

//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntake extends SubsystemBase {
  private SparkMax intakeMotor;
  private SparkMax indexerMotor;
  //public AbsoluteEncoder intakeEncoder;
  //public AbsoluteEncoder indexerEncoder;

 
  SparkMaxConfig config = new SparkMaxConfig();

  public GroundIntake() {
    intakeMotor = new SparkMax(0, MotorType.kBrushless);
    indexerMotor = new SparkMax(0, MotorType.kBrushless);
    config.idleMode(IdleMode.kBrake);
   // intakeEncoder = intakeMotor.getEncoder(); //didn't need encoder for intake/outtake
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void intakeOut() {
    intakeMotor.set(0.5);
  }

  public void intakeIn() {
    intakeMotor.set(-0.8);
  }

  public void stopIndexer() {
    indexerMotor.set(0);
  }

  public void indexerIn() {
    indexerMotor.set(-0.18);
  }

  public void indexerInSlow() {
    indexerMotor.set(-0.01);
  }

  public void indexerInFast() {
    indexerMotor.set(-0.4);
  }

  public void indexerOut() {
    indexerMotor.set(0.3);
  }

  public void stopIntakeIndexer() {
    stopIntake();
    stopIndexer();
  }

  public void inIntakeIndexer() {
    intakeIn();
    indexerIn();
  }

  public void outIntakeIndexer() {
    intakeOut();
    indexerOut();
  }

  public boolean isNote() {
    if (getLaserCanMeasurement() != null
        && getLaserCanMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return getLaserCanMeasurement().distance_mm / 1000.0 < Constants.AutoConstants.normalLaserCAN;
    }
    return false;
  }
}

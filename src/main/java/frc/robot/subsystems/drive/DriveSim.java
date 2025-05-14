package frc.robot.subsystems.drive;

import java.util.function.Consumer;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveSim extends Drive{
    private SwerveDriveSimulation driveSimulation;
    public DriveSim(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        super(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
        driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
    }
    public SwerveDriveSimulation getDriveSimulation(){
        return driveSimulation;
    }
    @Override
    public void setPose(Pose2d pose){
        driveSimulation.setSimulationWorldPose(pose);
        super.setPose(pose);
    }
}

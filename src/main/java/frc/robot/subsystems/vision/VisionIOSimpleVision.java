package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.Supplier;

public class VisionIOSimpleVision extends VisionIO {
  private final Supplier<Pose2d> poseSupplier;

  public VisionIOSimpleVision(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = true; // Always connected in this case
    inputs.tagIds = new int[1]; // To trick Vision.java into thinking we have a tag
    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              System.currentTimeMillis() / 1000.0,
              new Pose3d(poseSupplier.get()),
              0,
              100000,
              1,
              PoseObservationType.PHOTONVISION)
        };
  }
}

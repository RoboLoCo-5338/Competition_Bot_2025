package frc.robot.subsystems;

public abstract class IO<I extends IO.IOInputs> implements AutoCloseable {
  public static class IOInputs {
    public boolean connected = false;
  }

  public void updateInputs(I inputs) {}

  @Override
  public void close() {}
}

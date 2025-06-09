package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public abstract class IO implements AutoCloseable{
    public static class IOInputs {
        public boolean connected = false;
    }
    public void updateInputs(IOInputs inputs) {}
    @Override
    public void close() {}
}

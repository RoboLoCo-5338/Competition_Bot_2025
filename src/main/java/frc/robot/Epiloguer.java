package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.EpilogueConfiguration;

public final class Epiloguer {
  static EpilogueConfiguration config = Epilogue.getConfig();

  public static void bind(Robot robot) {
    if (config.loggingPeriod == null) {
      config.loggingPeriod = Seconds.of(robot.getPeriod());
    }
    if (config.loggingPeriodOffset == null) {
      config.loggingPeriodOffset = config.loggingPeriod.div(2);
    }

    robot.addPeriodicCallback(
        () -> {
          update(robot);
        });
  }

  /**
   * Updates Epilogue. This must be called periodically in order for Epilogue to record new values.
   * Alternatively, {@code bind()} can be used to update at an offset from the main robot loop.
   */
  public static void update(Robot robot) {
    System.out.println("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    long start = System.nanoTime();
    Epilogue.robotLogger.tryUpdate(
        config.backend.getNested(config.root), robot, config.errorHandler);
    config.backend.log("Epilogue/Stats/Last Run", (System.nanoTime() - start) / 1e6);
  }
}

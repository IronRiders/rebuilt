package edu.wpi.first.epilogue;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;

import org.ironriders.drive.DriveCommandsLogger;

public final class Epilogue {
  static {
    HAL.report(
      FRCNetComm.tResourceType.kResourceType_LoggingFramework,
      FRCNetComm.tInstances.kLoggingFramework_Epilogue
    );
  }

  private static final EpilogueConfiguration config = new EpilogueConfiguration();

  public static final DriveCommandsLogger driveCommandsLogger = new DriveCommandsLogger();

  public static void configure(java.util.function.Consumer<EpilogueConfiguration> configurator) {
    configurator.accept(config);
  }

  public static EpilogueConfiguration getConfig() {
    return config;
  }

  /**
   * Checks if data associated with a given importance level should be logged.
   */
  public static boolean shouldLog(Logged.Importance importance) {
    return importance.compareTo(config.minimumImportance) >= 0;
  }
}

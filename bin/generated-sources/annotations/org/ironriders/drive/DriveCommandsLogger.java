package org.ironriders.drive;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;

public class DriveCommandsLogger extends ClassSpecificLogger<DriveCommands> {
  private static final VarHandle $driveSubsystem;

  static {
    try {
      var lookup = MethodHandles.privateLookupIn(DriveCommands.class, MethodHandles.lookup());
      $driveSubsystem = lookup.findVarHandle(DriveCommands.class, "driveSubsystem", org.ironriders.drive.DriveSubsystem.class);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("[EPILOGUE] Could not load private fields for logging!", e);
    }
  }

  public DriveCommandsLogger() {
    super(DriveCommands.class);
  }

  @Override
  public void update(EpilogueBackend backend, DriveCommands object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
    }
  }
}

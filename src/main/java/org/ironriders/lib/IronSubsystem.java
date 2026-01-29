package org.ironriders.lib;

import java.util.Objects;
import java.util.concurrent.TimeUnit;

import org.ironriders.lib.Elastic.Notification;
import org.ironriders.lib.Elastic.NotificationLevel;

import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Common base for 4180 subsystems (mostly error/warning/debug messages and pushing stuff to the
 * dashboard).
 */
public abstract class IronSubsystem extends SubsystemBase {

    private final String diagnosticName =
            this.getClass().getSimpleName().replaceAll("Subsystem$", "");

    private final String dashboardPrefix = "Subsystems/" + diagnosticName + "/";
    private final String messagePrefix = diagnosticName + ": ";

    private final long startupTime;

    public IronSubsystem() {
        startupTime = System.nanoTime();
    }

    private String getThreadTime() {
        String timeStr = Objects.toString(
                TimeUnit.MILLISECONDS.convert(System.nanoTime() - startupTime, TimeUnit.NANOSECONDS)
                        / 1000d,
                "Error, could not get VM time!");

        return "[" + timeStr + "] ";
    }

    /**
     * Send a generic elastic notification with no edits.
     *
     * @param notif The elastic notification to send.
     */
    public void putNotification(Notification notif) {
        Elastic.sendNotification(notif);
    }

    /**
     * Send a elastic notification with level WARNING. This will also append your title to "warning
     * in (your subsystem): and set that as the title". You should be careful to put as little
     * information in the title as possible so it doesn't overflow. This notification will last 10
     * seconds
     *
     * @param notif The elastic notification to send.
     */
    public void notifyWarning(Notification notif) {
        notif.setLevel(NotificationLevel.WARNING);

        String title = notif.getTitle();
        title = "Warning in " + messagePrefix + ":" + title;

        notif.setTitle(title);
        notif.setDisplayTimeSeconds(10);
        
        putNotification(notif);
    }

    /**
     * Send a elastic notification with level info. This will also append your title to "Message from
     * (your subsystem): and set that as the title". You should be careful to put as little
     * information in the title as possible so it doesn't overflow. This notification will last 5
     * seconds
     *
     * @param notif The elastic notification to send.
     */
    public void notify(Notification notif) {
        notif.setLevel(NotificationLevel.INFO);

        String title = notif.getTitle();
        title = "Message from " + messagePrefix + ":" + title;

        notif.setTitle(title);
        notif.setDisplayTimeSeconds(5);
        
        putNotification(notif);
    }

    /**
     * Send a elastic notification with level ERROR. This will also append your title to "ERROR in
     * (your subsystem): and set that as the title". You should be careful to put as little
     * information in the title as possible so it doesn't overflow. This notification will last 30
     * seconds
     *
     * @param notif The elastic notification to send.
     */
    public void notifyError(Notification notif) {
        notif.setLevel(NotificationLevel.ERROR);

        String title = notif.getTitle();
        title = "ERROR in " + messagePrefix + ":" + title;

        notif.setTitle(title);
        notif.setDisplayTimeSeconds(30);

        putNotification(notif);
    }

    public void putTitleTextNotifcation(String title, String text) {
        Elastic.sendNotification(new Notification().withTitle(title).withDescription(text));
    }

    public void log(String msg) {
        putTitleTextNotifcation(getThreadTime() + messagePrefix, msg);
        DogLog.log(getThreadTime() + messagePrefix, msg);
    }

    /** Get a diagnostic value from SmartDashboard. */
    public double getDiagnostic(String name, double defaultValue) {
        return SmartDashboard.getNumber(name, defaultValue);
    }

    /**
     * DEBUGGING ONLY Publish a boolean diagnostic value to SmartDashboard with the prefix
     * `debug/Subsystems/{subsystem name}/`.
     */
    public void debugPublish(String name, boolean value) {
        SmartDashboard.putBoolean("debug/" + dashboardPrefix + name, value);
    }

    /**
     * DEBUGGING ONLY Publish a double diagnostic value to SmartDashboard with the prefix
     * `debug/Subsystems/{subsystem name}/`.
     */
    public void debugPublish(String name, double value) {
        SmartDashboard.putNumber("debug/" + dashboardPrefix + name, value);
    }

    /**
     * DEBUGGING ONLY Publish an String diagnostic value to SmartDashboard with the prefix
     * `debug/Subsystems/{subsystem name}/`.
     */
    public void debugPublish(String name, String value) {
        SmartDashboard.putString("debug/" + dashboardPrefix + name, value);
    }

    /**
     * DEBUGGING ONLY Publish a Sendable (including Commands) diagnostic value to SmartDashboard
     * with the prefix `debug/Subsystems/{subsystem name}/`. If the value is a Command, it will NOT also
     * be registered with PathPlanner's {@link NamedCommands}.
     */
    public void debugPublish(String name, Sendable value) {
        SmartDashboard.putData("debug/" + dashboardPrefix + name, value);
    }

    /**
     * Publish a boolean diagnostic value to SmartDashboard with the prefix `Subsystems/{subsystem
     * name}/`.
     */
    public void publish(String name, boolean value) {
        SmartDashboard.putBoolean(dashboardPrefix + name, value);
    }

    /**
     * Publish a double diagnostic value to SmartDashboard with the prefix `Subsystems/{subsystem
     * name}/`.
     */
    public void publish(String name, double value) {
        SmartDashboard.putNumber(dashboardPrefix + name, value);
    }

    /**
     * Publish an String diagnostic value to SmartDashboard with the prefix `Subsystems/{subsystem
     * name}/`.
     */
    public void publish(String name, String value) {
        SmartDashboard.putString(dashboardPrefix + name, value);
    }

    /**
     * Publish a Sendable (including Commands) diagnostic value to SmartDashboard with the prefix
     * `Subsystems/{subsystem name}/`. If the value is a Command, it will also be registered with
     * PathPlanner's {@link NamedCommands}.
     */
    public void publish(String name, Sendable value) {
        SmartDashboard.putData(dashboardPrefix + name, value);
        if (value instanceof Command) {
            NamedCommands.registerCommand(name, (Command) value);
        }
    }

    /**
     * Reports a String error message to {@link DriverStation#reportError(String, boolean)
     * DriverStation} with the time and subsystem name; Sends a notification to elastic with level
     * {@link NotificationLevel#ERROR ERROR}. with just the raw message.
     *
     * @param message The error message to report.
     */
    public void reportError(String message) {
        DriverStation.reportError(getThreadTime() + messagePrefix + message, false);
        Elastic.sendNotification(new Notification().withLevel(NotificationLevel.ERROR)
                .withTitle("ERROR").withDescription(message));
    }

    /**
     * Reports a String warning message to {@link DriverStation#reportWarning(String, boolean)
     * DriverStation} with the time and subsystem name; Sends a notification to
     * {@linkplain org.ironriders.lib.Elastic#sendNotification(Notification) Elastic} with level
     * {@link NotificationLevel#WARNING WARNING} with just the raw message.
     *
     * @param message The warning message to report.
     */
    public void reportWarning(String message) {
        DriverStation.reportWarning(getThreadTime() + messagePrefix + message, false);
        Elastic.sendNotification(new Notification().withLevel(NotificationLevel.WARNING)
                .withTitle("WARNING").withDescription(message));
    }
}

package org.ironriders.lib;

import java.util.concurrent.TimeUnit;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.Elastic.Notification;
import org.ironriders.lib.Elastic.NotificationLevel;

import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/*
 * This class implements many useful methods for a normal subsystem:
 * logging with the elastic dashboard and using DogLog,
 * common error handling methods.
 */
public class IronSubsystem extends SubsystemBase {
    /*
     * Make a string of the extender subsystems name, with the word "Subsystem"
     * replaced with "" (nothing) if it is at the end of the string.
     */
    private final String diagnosticName = this.getClass().getSimpleName().replaceAll("Subsystem$", "");

    /*
     * Create the actual path that the values published by this subsystem will be
     * placed at in the dashboard.
     * for example, lets say that FooSubsystem extends SubsystemHelper. In that
     * case, the path for data published by FooSubsystem would be "Subsystems/Foo/"
     */
    private final String dashboardPath = "Subsystems/" + diagnosticName + "/";

    // This value will hold the time the subsystem was constructed, useful for
    // logging.
    private long startupTime;

    // A simple constructor, just gets the system time.
    public IronSubsystem() {
        startupTime = System.nanoTime();
    }

    // ---- Methods ----
    // -- Scheduling --

    public static void schedule(Command command) {
        CommandScheduler.getInstance().schedule(command);
    }

    // -- Logging: Putting objects to the display field --

    /**
     * Get or create an object with some name.
     * 
     * @param name The name of the object.
     * 
     * @return The new object.
     */
    public FieldObject2d putObject(String name) {
        if (DriveSubsystem.getSwerveDrive().field == null) {
            throw new RuntimeException("Tried to put object '" + name + "' to the field before the SwerveDrive was initialized!");
        }

        return DriveSubsystem.getSwerveDrive().field.getObject(name);
    }

    /**
     * Put a Pose2d to the field.
     */
    public void putPose2d(Pose2d pose, String name) {
        putObject(name).setPose(pose);
    }

    // -- Logging: Notifications --
    /*
     * This method returns the time the subsystem has been running for.
     */
    public Double getRunningTime() {
        // Convert the nanosecond value of the current time to milliseconds, then divide
        // by 1000d to get seconds with a pleasing number of decimals.
        return TimeUnit.MILLISECONDS.convert(System.nanoTime() - startupTime, TimeUnit.NANOSECONDS) / 1000d;
    }

    /*
     * This method formats the subsystem's running time nicely.
     */
    public String getTimeString() {
        // Add square braces.
        return String.format("[%s]", String.valueOf(getRunningTime()));
    }

    /*
     * This method gets the prefix to be printed before the actual text of a log.
     * e.g. [2.7] FooSubsystem: Bar.
     * ^------------------^
     * Message Prefix: ^
     */
    public String getMessagePrefix() {
        return String.format("%s %s: ", getTimeString(), diagnosticName);
    }

    /*
     * This method gets the prefix to be printed before the actual text of a log without including the time.
     * e.g. FooSubsystem: Bar.
     *      ^------------^
     * Message Prefix: ^
     */
    public String getTimelessMessagePrefix() {
        return String.format("%s: ", diagnosticName);
    }

    /**
     * Send a generic Elastic notification with no edits.
     *
     * @param notification The Elastic notification to send.
     */
    public void showNotification(Notification notification) {
        Elastic.sendNotification(notification);
    }

    /**
     * Send a elastic notification with level INFO.
     * This is the default notification level. 
     * 
     * @param notification The Elastic notification to send.
     */
    public void notifyInfo(Notification notification) {
        notification.setLevel(NotificationLevel.INFO);
        showNotification(notification);
    }

    /**
     * Send a elastic notification with level WARNING. 
     * 
     * @param notification The Elastic notification to send.
     */
    public void notifyWarning(Notification notification) {
        notification.setLevel(NotificationLevel.WARNING);
        showNotification(notification);
    }

     /**
     * Send a elastic notification with level ERROR. 
     * 
     * @param notification The Elastic notification to send.
     */
    public void notifyError(Notification notification) {
        notification.setLevel(NotificationLevel.ERROR);
        showNotification(notification);
    }

    /**
     * Show a notification with both a title and body.
     * 
     * @param title The title of the notification
     * @param body The body text of the notification
     */
    public void showParagraphNotification(String title, String body) {
      showNotification(new Notification(title, body));
    }

    /**
     * Utility function to make a notification with the given title, concatenating the subsystem's log value to the beginning.
     * 
     * @param title The title of the notification, with the subsystem's log value added to the beginning
     * @param body The body text of the notification
     */
    public Notification buildNotification(String title, String body) {
        return new Notification(getTimelessMessagePrefix() + title, body);
    }

    /**
     * Method to call the builder and then show the resulting notification
     * 
     * @param title The title of the notification, with the subsystem's log value added to the beginning
     * @param body The body text of the notification
     */
    public void showInfoNotification(String title, String body) {
        showNotification(buildNotification(title, body));
    }

    // -- Logging: SmartDashboard Publishers --

    /**
     * DEBUGGING ONLY.
     * Publish a Boolean diagnostic value to SmartDashboard with the prefix
     * `debug/Subsystems/{Subsystem Name}/`.
     */
    public void debugPublish(String name, Boolean value) {
        SmartDashboard.putBoolean("debug/" + dashboardPath + name, value);
    }

    /**
     * DEBUGGING ONLY.
     * Publish a Double diagnostic value to SmartDashboard with the prefix
     * `debug/Subsystems/{Subsystem Name}/`.
     */
    public void debugPublish(String name, Double value) {
        SmartDashboard.putNumber("debug/" + dashboardPath + name, value);
    }

    /**
     * DEBUGGING ONLY.
     * Publish a String diagnostic value to SmartDashboard with the prefix
     * `debug/Subsystems/{Subsystem Name}/`.
     */
    public void debugPublish(String name, String value) {
        SmartDashboard.putString("debug/" + dashboardPath + name, value);
    }

    /**
     * DEBUGGING ONLY.
     * Publish a {@link Sendable} (including commands) as a diagnostic value to SmartDashboard
     * with the prefix `debug/Subsystems/{Subsystem Name}/`.
     * 
     * If the value is a {@link Command}, it will NOT also
     * be registered with PathPlanner's {@link NamedCommands}.
     */
    public void debugPublish(String name, Sendable value) {
        SmartDashboard.putData("debug/" + dashboardPath + name, value);
    }

    /**
     * Publish a Boolean diagnostic value to SmartDashboard with the prefix
     * `Subsystems/{Subsystem Name}/`.
     */
    public void publish(String name, Boolean value) {
        SmartDashboard.putBoolean(dashboardPath + name, value);
    }

    /**
     * Publish a Double diagnostic value to SmartDashboard with the prefix
     * `Subsystems/{Subsystem Name}/`.
     */
    public void publish(String name, Double value) {
        SmartDashboard.putNumber(dashboardPath + name, value);
    }

    public void publish(String name, String value) {
        SmartDashboard.putString(dashboardPath + name, value);
    }

    /**
     * Read a Boolean diagnostic value from SmartDashboard with the prefix
     * `Subsystems/{Subsystem Name}/`.
     */
    public boolean getPublishedBoolean(String name, boolean defaultValue) {
        return SmartDashboard.getBoolean(dashboardPath + name, defaultValue);
    }

    /**
     * Read a Double diagnostic value from SmartDashboard with the prefix
     * `Subsystems/{Subsystem Name}/`.
     */
    public double getPublishedNumber(String name, double defaultValue) {
        return SmartDashboard.getNumber(dashboardPath + name, defaultValue);
    }

    /**
     * Read a String diagnostic value from SmartDashboard with the prefix
     * `Subsystems/{Subsystem Name}/`.
     */
    public String getPublishedString(String name, String defaultValue) {
        return SmartDashboard.getString(dashboardPath + name, defaultValue);
    }

    /**
     * Publish a {@link Sendable} (including commands) as a diagnostic value to SmartDashboard
     * with the prefix `Subsystems/{Subsystem Name}/`.
     * 
     * If the value is a {@link Command}, it will also
     * be registered with PathPlanner's {@link NamedCommands}.
     */
    public void publish(String name, Sendable value) {
        SmartDashboard.putData(dashboardPath + name, value);

        if (value instanceof Command) {
            // Named commands are commands that are callable from a PathPlanner auto.
            // See PathPlannerExamples.java or the PathPlanner docs.
            NamedCommands.registerCommand(name, (Command) value);
        }
    }

    /**
     * Reports a String error message to
     * {@link DriverStation#reportError} with the time and subsystem name; Sends a notification to
     * elastic with level {@link NotificationLevel#ERROR} with the message.
     *
     * @param message The error message to report.
     */
    public void reportError(String message) {
        DriverStation.reportError(getMessagePrefix() + message, false);

        Elastic.sendNotification(new Notification().withLevel(NotificationLevel.ERROR)
                .withTitle("ERROR").withDescription(message));
    }

    /**
     * Reports a String warning message to
     * {@link DriverStation#reportError} with the time and subsystem name; Sends a notification to
     * elastic with level {@link NotificationLevel#WARNING} with the message.
     *
     * @param message The error message to report.
     */
    public void reportWarning(String message) {
        DriverStation.reportWarning(getMessagePrefix() + message, false);

        Elastic.sendNotification(new Notification().withLevel(NotificationLevel.WARNING)
                .withTitle("WARNING").withDescription(message));
    }

    // -- Logging: DogLog --

    /**
     * Log a message to "Subsystems/Foo/{name}" with the contents of message
     * 
     * @param name The name of the logged value, e.g. catsPet
     * @param message The message to log, e.g. 4
     */
    public void log(String name, String message) {
        DogLog.log(dashboardPath + name, message);
    }
}

package org.ironriders.core;

import static org.ironriders.lib.BallisticsUtils.getPosition;
import static org.ironriders.lib.BallisticsUtils.inRange;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.BallisticsUtils;
import org.ironriders.lib.DriverRequest;
import org.ironriders.lib.DriverRequest.AlignTargetingMode;
import org.ironriders.lib.DriverRequest.LauncherTargetingMode;
import org.ironriders.lib.DriverRequest.PriorityMode;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.FieldPositions;
import org.ironriders.manipulation.launcher.LauncherConstants.State;
import org.ironriders.manipulation.launcher.LauncherSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/*
 * Class holding state for targeting points with launcher and drive angles.
 */
public class TargetingControl {
    private static Pose3d launcherTarget = FieldPositions.get(ElementType.HUB);
    private static double alignTarget = 0;

    private static DriverRequest request = new DriverRequest(PriorityMode.DRIVER_PRIORITY,
            AlignTargetingMode.LAUNCHER, LauncherTargetingMode.HUB);

    private static DriverRequest lastDriverRequest = request;

    private static PriorityMode lastAppliedPriorityMode = null;

    public static void receiveRequest(DriverRequest driverRequest) {
        lastDriverRequest = request;
        request = driverRequest;
        update();
    }

    public static void revert() {
        request = lastDriverRequest;
        update();
    }

    /**
     * Revert the targeting mode to safe default values,
     * Driver has control,
     * Launcher has control over auto align,
     * Target the hub.
     */
    public static void revertToSafeDefaults() {
        lastDriverRequest = request;
        request = new DriverRequest(PriorityMode.DRIVER_PRIORITY,
                AlignTargetingMode.LAUNCHER, LauncherTargetingMode.HUB);
        update();
    }

    public static void targetHub() {
        new DriverRequest(PriorityMode.LAUNCHER_PRIORITY,
                AlignTargetingMode.LAUNCHER, LauncherTargetingMode.HUB).send("target hub");
    }

    public static Command targetHubCommand() {
        return Commands.runOnce(() -> targetHub());
    }

    public static void targetPassing() {
        new DriverRequest(PriorityMode.LAUNCHER_PRIORITY,
                AlignTargetingMode.LAUNCHER, LauncherTargetingMode.PASSING).send("target passing");
    }

    public static Command targetPassingCommand() {
        return Commands.runOnce(() -> targetPassing());
    }

    /*
     * Update the state of everything relevant to targeting. Should be called every
     * tick.
     */
    public static void update() {
        if (request.m_launcherTargetingMode == LauncherTargetingMode.HUB) {
            launcherTarget = FieldPositions.get(ElementType.HUB);
        }

        /*
         * We need to:
         * Update the launcher target:
         * if launcher target is hub, set the target to the hub, no need to do any
         * processing.
         * else if launcher target is passing, set the target to the closest point in
         * the passing zone that is in range.
         * 
         * Update the alignment target:
         * if align target is bump, set to bump.
         * else if align target is launcher, find the angle to the launcher target and
         * set it.
         * 
         * Check what mode we are in:
         * if we are in driver priority then make no command,
         * else if we are in launcher priority, command drive to face launcher target,
         * else if we are in align priority, command drive to face align target.
         */

        alignTarget = getAlignTarget();

        LauncherSubsystem.setTarget(launcherTarget);

        DriveSubsystem.setRotationGoalRad(alignTarget);
        
        boolean priorityChanged = (request.m_priorityMode != lastAppliedPriorityMode);
        lastAppliedPriorityMode = request.m_priorityMode;

        switch (request.m_priorityMode) {
            default:
            case DRIVER_PRIORITY:
                // We want the driver to have control, disable PID control.
                DriveSubsystem.setPIDRotationControl(false);
                // Set the launcher to idle (1/2 max speed).
                if (priorityChanged) {
                    RobotContainer.launcherSubsystem.setCurrentState(State.IDLE);
                }
                return;
            case LAUNCHER_PRIORITY:
                // Get the launcher ready (set to max speed).
                if (priorityChanged) {
                    RobotContainer.launcherSubsystem.setCurrentState(State.READY);
                }
                // Fall though here as we want to use targeting whether or not we are launching.
            case ALIGN_PRIORITY:
                // We want the targeting system to have control, enable PID control.
                DriveSubsystem.setPIDRotationControl(true);
                return;
        }
    }

    /**
     * Calculate the angle we should face for the align mode of the request.
     * 
     * @return The angle in degrees.
     */
    private static double getAlignTarget() {
        switch (request.m_alignTargetingMode) {
            default:
            case LAUNCHER:
                SmartDashboard.putNumber("Angle",
                        Utils.getAngleToPointRadians(DriveSubsystem.getPose(), launcherTarget.toPose2d()) + Math.PI);
                return Utils.getAngleToPointRadians(DriveSubsystem.getPose(), launcherTarget.toPose2d()) + Math.PI;

            case OUTPOST:
                return Math.PI;

            case BUMP:
                return Math.toRadians(findClosest45DegreeAngle(DriveSubsystem.getPose().getRotation().getDegrees()));
        }
    }

    /**
     * Find the nearest 45 degree angle that is not axis aligned (for going over the
     * bump) from a given angle.
     * 
     * @param angle Our current angle in degrees.
     * @return The best angle in degrees.
     */
    private static double findClosest45DegreeAngle(double angle) {
        angle = Math.toRadians(angle);

        angle = angle % (2 * Math.PI);
        if (angle < 0) {
            angle += 2 * Math.PI;
        }

        for (Double i = 0.25; i <= 2; i += 0.5) {
            if (Math.abs(angle - (i * Math.PI)) < 0.25 * Math.PI) {
                return Math.toDegrees(i * Math.PI);
            }
        }

        return Math.toDegrees(0.25 * Math.PI);
    }
}
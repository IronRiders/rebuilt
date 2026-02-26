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

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/*
 * Class holding state for targeting points with launcher and drive angles.
 */
public class TargetingControl {
    private static Pose3d launcherTarget;
    private static double alignTarget;

    private static DriverRequest request = new DriverRequest(PriorityMode.DRIVER_PRIORITY,
            AlignTargetingMode.LAUNCHER, LauncherTargetingMode.HUB);

    private static DriverRequest lastDriverRequest = request;

    public static void receiveRequest(DriverRequest driverRequest) {
        lastDriverRequest = request;
        request = driverRequest;
    }

    public static void revert() {
        request = lastDriverRequest;
        update();
    }

    public static void revertToSafeDefaults() {
        lastDriverRequest = request;
        request = new DriverRequest(PriorityMode.DRIVER_PRIORITY,
                AlignTargetingMode.LAUNCHER, LauncherTargetingMode.HUB);
    }

    public static void targetHubInternal() {
        new DriverRequest(PriorityMode.LAUNCHER_PRIORITY,
                AlignTargetingMode.LAUNCHER, LauncherTargetingMode.HUB).send("target hub internal");
    }

    public static Command targetHub() {
        return Commands.runOnce(() -> targetHubInternal());
    }

    public static void targetPassingInternal() {
        new DriverRequest(PriorityMode.LAUNCHER_PRIORITY,
                AlignTargetingMode.LAUNCHER, LauncherTargetingMode.PASSING).send("target passing internal");
    }

    public static Command targetPassing() {
        return Commands.runOnce(() -> targetPassingInternal());
    }

    /*
     * Update the state of everything relevant to targeting. Should be called every
     * tick.
     */
    public static void update() {
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

        launcherTarget = getLauncherTarget();

        alignTarget = getAlignTarget();

        LauncherSubsystem.currentTarget = launcherTarget;
        DriveSubsystem.setRotationGoal(alignTarget);

        switch (request.r_priorityMode) {
            default:
            case DRIVER_PRIORITY:
                DriveSubsystem.setPIDRotationControl(false);
                LauncherSubsystem.currentState = State.IDLE;

                break;
            // return;

            case ALIGN_PRIORITY:
            case LAUNCHER_PRIORITY:
                DriveSubsystem.setPIDRotationControl(true);
                LauncherSubsystem.currentState = State.READY;

                break;
            // return;
        }

        DogLog.log("TargetingValues",
                "PID Control: " + String.valueOf(DriveSubsystem.PIDRotation) + " LauncherTargetMode: "
                        + request.r_launcherTargetingMode.toString() + " AlignTargetMode: "
                        + String.valueOf(request.r_alignTargetingMode));
    }

    private static double getAlignTarget() {
        switch (request.r_alignTargetingMode) {
            default:
            case LAUNCHER:
                return Utils.getAngleToPoint(DriveSubsystem.getPose(), Utils.flattenPose3d(launcherTarget)) + 180;

            case OUTPOST:
                return 180;

            case BUMP:
                return Math.toDegrees(findClosest45DegreeAngleInRadians(getPosition().getRotation().getRadians()));
        }
    }

    private static Pose3d getLauncherTarget() {
        switch (request.r_launcherTargetingMode) {
            default:
            case HUB:
                return FieldPositions.get(ElementType.HUB);

            case PASSING:
                Pose2d closest = getPosition().nearest(FieldPositions.Zones.PASSING_POINTS);
                Pose2d bestPoint = closest;

                if (!inRange(Utils.expandPose2d(bestPoint))) {
                    Translation2d t = BallisticsUtils.translationToPoint(getPosition(),
                            closest, LauncherSubsystem.range[1]);

                    bestPoint = new Pose2d(t.getX() + getPosition().getX(), t.getY() + getPosition().getY(),
                            new Rotation2d());
                }

                return Utils.expandPose2d(bestPoint);
        }
    }

    private static double findClosest45DegreeAngleInRadians(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle < 0) {
            angle += 2 * Math.PI;
        }

        for (Double i = 0.25; i <= 2; i += 0.5) {
            if (Math.abs(angle - (i * Math.PI)) < 0.25 * Math.PI) {
                return i * Math.PI;
            }
        }

        return 0.25 * Math.PI;
    }
}
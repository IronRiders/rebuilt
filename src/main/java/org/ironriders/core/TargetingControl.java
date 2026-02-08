package org.ironriders.core;

import org.ironriders.core.DriverRequest.AlignTargetingMode;
import org.ironriders.core.DriverRequest.LauncherTargetingMode;
import org.ironriders.core.DriverRequest.PriorityMode;
import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.manipulation.launcher.LauncherSubsystem;
import org.ironriders.manipulation.launcher.LauncherConstants.State;

import dev.doglog.DogLog;

import org.ironriders.lib.field.FieldPositions;

import edu.wpi.first.math.geometry.Pose3d;
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
                DriveSubsystem.setPIDControl(false);
                LauncherSubsystem.currentState = State.IDLE;

                break;
            // return;

            case ALIGN_PRIORITY:
            case LAUNCHER_PRIORITY:
                DriveSubsystem.setPIDControl(true);
                LauncherSubsystem.currentState = State.READY;

                break;
            // return;
        }

        DogLog.log("TargetingValues",
                "PID Control: " + String.valueOf(DriveSubsystem.PIDAlign) + " LauncherTargetMode: "
                        + request.r_launcherTargetingMode.toString() + " AlignTargetMode: "
                        + String.valueOf(request.r_alignTargetingMode));
    }

    private static double getAlignTarget() {
        switch (request.r_alignTargetingMode) {
            default:
            case LAUNCHER:
                return Utils.getAngleToPoint(DriveSubsystem.getPose(), Utils.flattenPose3d(launcherTarget));

            case OUTPOST:
                return 180;

            case BUMP:
                return 45;
        }
    }

    private static Pose3d getLauncherTarget() {
        switch (request.r_launcherTargetingMode) {
            default:
            case HUB:
                return FieldPositions.get(ElementType.HUB);

            case PASSING:
                return RobotContainer.passingZone.closestPointAsPose3d();
        }
    }
}
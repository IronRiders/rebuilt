package org.ironriders.core;

import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.FieldPositions;

import edu.wpi.first.math.geometry.Pose3d;

/*
 * Class holding state for targeting points with launcher and drivebase angles.
 */
public class TargetingControl {
    enum targetingMode {
        DRIVER_PRIORITY(),
        LAUNCHER_PRIORITY(),
        ALIGN_PRIORITY();
    }

    enum alignDegreesTargets {
        NORMAL(0d),
        BUMP(45d);

        public double rotation;
        alignDegreesTargets(double rotation) {
            this.rotation = rotation;
        }
    }

    enum launcherTarget {
        HUB(),
        PASSING();
    }

    private static Pose3d launcherTarget = new Pose3d();

    private static targetingMode currentMode = targetingMode.DRIVER_PRIORITY;

    private static Pose3d getPoseForTarget(launcherTarget target) {
        switch (target) {
            case HUB:
                return FieldPositions.get(ElementType.HUB);

            case PASSING:
                return RobotContainer.passingZone.closestPointAsPose3d();

            default:
                return new Pose3d();
        }
    }

    public static void setMode(targetingMode mode) {
        currentMode = mode;
    }

    public static Pose3d getLauncherTarget() {
        return launcherTarget;
    }
}

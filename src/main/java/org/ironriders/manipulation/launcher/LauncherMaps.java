package org.ironriders.manipulation.launcher;

import java.util.List;

import org.ironriders.lib.InterpolatingDoubleMap;

public class LauncherMaps {
    public DistanceToAngleMap distanceToAngleMap;
    public DistanceToFlyWheelSpeedMap distanceToFlyWheelSpeedMap;
    public DistanceToExtensionMap distanceToExtensionMap;

    /*
     * Setup maps.
     */
    public LauncherMaps() {
        this.distanceToAngleMap = new DistanceToAngleMap();
        this.distanceToFlyWheelSpeedMap = new DistanceToFlyWheelSpeedMap();
        this.distanceToExtensionMap = new DistanceToExtensionMap();

    }

    public class DistanceToFlyWheelSpeedMap {
        public static InterpolatingDoubleMap distanceToFlyWheelSpeedMap = new InterpolatingDoubleMap();

        DistanceToFlyWheelSpeedMap() {
            distanceToFlyWheelSpeedMap.put(2.408, 34.97);
            distanceToFlyWheelSpeedMap.put(3.13, 36.15);
            distanceToFlyWheelSpeedMap.put(3.97, 39.30);
            distanceToFlyWheelSpeedMap.put(4.6, 39.53);
            distanceToFlyWheelSpeedMap.put(10d, 40d);

        }

        public static double getFlyWheelSpeedForDistance(double distance) {
            return distanceToFlyWheelSpeedMap.get(distance);
        }

    }

    public class DistanceToAngleMap {
        public static InterpolatingDoubleMap distanceToAngleMap = new InterpolatingDoubleMap();

        DistanceToAngleMap() {
            distanceToAngleMap.put(0d, 0d); // TODO: replace with real data

        }

        public static double getAngleForDistance(double distance) {
            return distanceToAngleMap.get(distance);
        }

    }

    public class DistanceToExtensionMap {
        public static InterpolatingDoubleMap distanceToExtensionMap = new InterpolatingDoubleMap();

        DistanceToExtensionMap() {
            distanceToExtensionMap.put(2.408, 0.31);
            distanceToExtensionMap.put(3.13, .42);
            distanceToExtensionMap.put(3.97, .42);
            distanceToExtensionMap.put(4.6, 0.50);
            distanceToExtensionMap.put(10d, 0.50);

        }

        public static double getExtensionForDistance(double distance) {
            return distanceToExtensionMap.get(distance);
        }

    }

}

package org.ironriders.manipulation.launcher;

import org.ironriders.lib.InterpolatingDoubleMap;

import dev.doglog.DogLog;

public class LauncherMaps {
    public AngleToExtensionMap angleToExtensionMap;

    /*
     * Setup maps.
     */
    public LauncherMaps() {
        this.angleToExtensionMap = new AngleToExtensionMap();
    }

    public class AngleToExtensionMap {
        public static InterpolatingDoubleMap angleToExtensionMap = new InterpolatingDoubleMap();

        // See: https://tinyurl.com/3smfzd2v and
        // https://wcproducts.com/products/wcp-0415 for info.
        public static double strokeLength = 140; // mm
        public static double maxExtension = strokeLength * 2;

        AngleToExtensionMap() {
            /* Key: Angle, Value: Extension Amount (mm) */
            // Values currently from CAD. TODO
            angleToExtensionMap.put(-1d, 0d);
            angleToExtensionMap.put(0d, 113.8d);
            angleToExtensionMap.put(1d, 105.4d);
            angleToExtensionMap.put(90d, maxExtension);
        }

        /*
         * Get the value in terms of 0-1 instead of raw millimeters.
         * You should probably use this method in most cases.
         */
        public static double getExtensionForAngle(double angle) {
            return angleToExtensionMap.get(angle) / maxExtension;
        }

        /*
         * Get the angle for the given extension (0-1).
         */
        public static double getAngleForExtension(double extension) {
            DogLog.log("get key", String.valueOf(extension * maxExtension));
            return angleToExtensionMap.getKeysByValue(extension * maxExtension).orElseThrow().get(0);
        }
    }
}

package org.ironriders.manipulation.launcher;

import org.ironriders.lib.InterpolatingDoubleMap;

public class LauncherMaps {
    public AngleToExtensionMap angleToExtensionMap;

    /*
     * Setup maps.
     */
    public LauncherMaps() {
        this.angleToExtensionMap = new AngleToExtensionMap();
    }

    /*
     * InterpolatingDoubleTreeMap interpolationMapAngle = new
     * InterpolatingDoubleTreeMap();
     * interpolationMapAngle.put(null, null);
     * interpolationMapAngle.put(null, null);
     * interpolationMapAngle.put(null, null);
     * interpolationMapAngle.put(null, null);
     * interpolationMapAngle.put(null, null);
     * interpolationMapAngle.put(null, null);
     * interpolationMapAngle.put(null, null);
     * interpolationMapAngle.put(null, null);
     * 
     * InterpolatingDoubleTreeMap interpolationMapVelocity = new
     * InterpolatingDoubleTreeMap();
     * interpolationMapVelocity.put(null, null);
     * interpolationMapVelocity.put(null, null);
     * interpolationMapVelocity.put(null, null);
     * interpolationMapVelocity.put(null, null);
     * interpolationMapVelocity.put(null, null);
     * interpolationMapVelocity.put(null, null);
     * interpolationMapVelocity.put(null, null);
     */

    public class AngleToExtensionMap {
        public static InterpolatingDoubleMap angleToExtensionMap = new InterpolatingDoubleMap();

        // See: https://tinyurl.com/3smfzd2v and
        // https://wcproducts.com/products/wcp-0415 for info.
        public static double strokeLength = 140; // mm
        public static double maxExtension = strokeLength * 2;

        AngleToExtensionMap() {
            /* Key: Angle, Value: Extension Amount (mm) */
            // Values currently from CAD. TODO
            angleToExtensionMap.put(0d, 5d);

        }

        /*
         * Get the value in terms of 0-1 instead of raw millimeters.
         * You should probably use this method in most cases.
         */
        public static double getExtensionForAngle(double angle) {
            return angleToExtensionMap.get(angle) / maxExtension;
        }

        public static double getAngleForExtension(double extension) {
            return angleToExtensionMap.getKeysByValue(extension).orElseThrow().get(0);
        }
    }
}

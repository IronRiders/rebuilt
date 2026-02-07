package org.ironriders.lib;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class InterpolatingDoubleMap extends InterpolatingMap<Double, Double> {
    public InterpolatingDoubleMap() {
        super(InverseInterpolator.forDouble(), Interpolator.forDouble());
    }

    /**
     * Creates an {@link InterpolatingDoubleTreeMap} from the given entries.
     *
     * @param entries The entries to add to the map.
     * @return The map filled with the {@code entries}.
     */
    @SafeVarargs
    public static InterpolatingDoubleTreeMap ofEntries(Map.Entry<Double, Double>... entries) {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        for (var entry : entries) {
            map.put(entry.getKey(), entry.getValue());
        }
        return map;
    }
}

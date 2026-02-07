package org.ironriders.lib;

import java.util.Collection;
import java.util.Comparator;
import java.util.List;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.TreeMap;
import java.util.function.Predicate;

import dev.doglog.DogLog;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/*
 * Re-implementation of WPILib InterpolatingTreeMap to expose more features of the tree map.
 */
public class InterpolatingMap<K extends Number, V extends Number> {
    public TreeMap<K, V> map = new TreeMap<>();

    private final InverseInterpolator<K> inverseInterpolator;
    private final Interpolator<V> interpolator;

    public InterpolatingMap(InverseInterpolator<K> _inverseInterpolator, Interpolator<V> _interpolator) {
        inverseInterpolator = _inverseInterpolator;
        interpolator = _interpolator;
    }

    /**
     * Inserts a key-value pair.
     *
     * @param key   The key.
     * @param value The value.
     */
    public void put(K key, V value) {
        map.put(key, value);
    }

    /**
     * Returns the value associated with a given key.
     *
     * <p>
     * If there's no matching key, the value returned will be an interpolation
     * between the keys
     * before and after the provided one, using the {@link Interpolator} and {@link
     * InverseInterpolator} provided.
     *
     * @param key The key.
     * @return The value associated with the given key.
     */
    public V get(K key) {
        V val = map.get(key);
        if (val == null) {
            K ceilingKey = map.ceilingKey(key);
            K floorKey = map.floorKey(key);

            if (ceilingKey == null && floorKey == null) {
                return null;
            }
            if (ceilingKey == null) {
                return map.get(floorKey);
            }
            if (floorKey == null) {
                return map.get(ceilingKey);
            }
            V floor = map.get(floorKey);
            V ceiling = map.get(ceilingKey);

            return interpolator.interpolate(
                    floor, ceiling, inverseInterpolator.inverseInterpolate(floorKey, ceilingKey, key));
        } else {
            return val;
        }
    }

    public boolean isKeyPresent(K key) {
        return map.containsKey(key);
    }

    public boolean isValuePresent(V value) {
        return map.containsValue(value);
    }

    public Collection<V> values() {
        return map.values();
    }

    public Set<K> keys() {
        return map.keySet();
    }

    public Set<Entry<K, V>> entries() {
        return map.entrySet();
    }

    /*
     * Switch keys and values, then return the new TreeMap.
     */
    public TreeMap<V, K> swap() {
        TreeMap<V, K> newMap = new TreeMap<V, K>();

        for (Entry<K, V> entry : map.entrySet()) {
            newMap.put(entry.getValue(), entry.getKey());
        }

        return newMap;
    }

    /*
     * Will return a list of keys for a given value. If no keys are found, the will
     * be interpolated. If the value is greater that the greatest value or less than
     * the least, will return an Optional.empty().
     */
    public Optional<List<K>> getKeysByValue(V value) {
        DogLog.log("Value", String.valueOf(value.doubleValue()));

        List<Entry<K, V>> entriesList = map.entrySet().stream().sorted(new Comparator<Entry<K, V>>() {
            @Override
            // Sort by values.
            public int compare(Entry<K, V> a, Entry<K, V> b) {
                if (Objects.equals(a.getValue(), b.getValue())) {
                    return 0;
                }

                return (Double) a.getValue() > (Double) b.getValue() ? 1 : -1;
            }
        }).toList();

        List<K> keysList = entriesList.stream().filter(new Predicate<Entry<K, V>>() {
            public boolean test(Entry<K, V> entry) {
                return Objects.equals(entry.getValue(), value);
            };
        }).map((Entry<K, V> e) -> e.getKey()).toList();

        switch (keysList.size()) {
            case 0: // Zero keys, interpolate.
                // Find adjacent entries (by value) that span the requested value and
                // interpolate the key.
                for (int i = 0; i < entriesList.size() - 1; i++) {
                    Entry<K, V> a = entriesList.get(i);
                    Entry<K, V> b = entriesList.get(i + 1);

                    DogLog.log("Ceiling", a.toString());
                    DogLog.log("Floor", a.toString());

                    double va = a.getValue().doubleValue();
                    double vb = b.getValue().doubleValue();
                    double ka = a.getKey().doubleValue();
                    double kb = b.getKey().doubleValue();

                    DogLog.log("Values and Keys",
                            "Values: Ceiling: " + String.valueOf(va) + " Floor: " + String.valueOf(vb)
                                    + " Keys: Ceiling: " + String.valueOf(ka) + " Floor: " + String.valueOf(kb));

                    if (Utils.inRange(va, vb, value.doubleValue())) {
                        double t = (value.doubleValue() - va) / (vb - va);
                        double interpolatedKey = lerp(ka, kb, t);
                        DogLog.log("interpolated key", String.valueOf(interpolatedKey));

                        @SuppressWarnings("unchecked")
                        K resultKey = fromDouble(interpolatedKey, (Class<K>) a.getKey().getClass());
                        return Optional.of(List.of(resultKey));
                    }
                }

                return Optional.empty();

            default: // One or more keys for the value
                return Optional.of(keysList);
        }
    }

    /** Clears the contents. */
    public void clear() {
        map.clear();
    }

    private double lerp(double start, double end, double t) {
        return start + t * (end - start);
    }

    @SuppressWarnings("unchecked")
    public K fromDouble(double value, Class<K> type) {
        if (type == Double.class) {
            return (K) Double.valueOf(value);
        } else if (type == Float.class) {
            return (K) Float.valueOf((float) value);
        } else if (type == Integer.class) {
            return (K) Integer.valueOf((int) value);
        } else if (type == Long.class) {
            return (K) Long.valueOf((long) value);
        } else if (type == Short.class) {
            return (K) Short.valueOf((short) value);
        } else if (type == Byte.class) {
            return (K) Byte.valueOf((byte) value);
        }
        throw new IllegalArgumentException("Unsupported numeric type: " + type);
    }
}

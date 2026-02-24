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
    @SuppressWarnings("null")
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
    @SuppressWarnings("null")
    public Optional<List<K>> getKeysByValue(V value) {
        List<Entry<K, V>> entriesList = map.entrySet().stream().sorted(new Comparator<Entry<K, V>>() {
            @SuppressWarnings("null")
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
            @SuppressWarnings("null")
            public boolean test(Entry<K, V> entry) {
                return entry.getValue().doubleValue() == value.doubleValue();
            };
        }).map((Entry<K, V> e) -> e.getKey()).toList();

        switch (keysList.size()) {
            case 0: // Zero keys, interpolate.
                // Find adjacent entries (by value) that span the requested value and
                // interpolate the key.
                for (int i = 0; i < entriesList.size() - 1; i++) {
                    Entry<K, V> a = entriesList.get(i);
                    Entry<K, V> b = entriesList.get(i + 1);

                    @SuppressWarnings("null")
                    double va = a.getValue().doubleValue();
                    @SuppressWarnings("null")
                    double vb = b.getValue().doubleValue();

                    @SuppressWarnings("null")
                    double ka = a.getKey().doubleValue();
                    @SuppressWarnings("null")
                    double kb = b.getKey().doubleValue();

                    if (Utils.inRange(va, vb, value.doubleValue())) {
                        double t = (value.doubleValue() - va) / (vb - va);

                        @SuppressWarnings({ "unchecked", "null" })
                        double interpolatedKey = (double) interpolator.interpolate(
                                VfromDouble(ka, (Class<V>) a.getValue().getClass()),
                                VfromDouble(kb, (Class<V>) a.getValue().getClass()), t);

                        @SuppressWarnings({ "unchecked", "null" })
                        K resultKey = KfromDouble(interpolatedKey, (Class<K>) a.getKey().getClass());

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

    @SuppressWarnings("unchecked")
    public K KfromDouble(double value, Class<K> type) {
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

    @SuppressWarnings("unchecked")
    public V VfromDouble(double value, Class<V> type) {
        if (type == Double.class) {
            return (V) Double.valueOf(value);
        } else if (type == Float.class) {
            return (V) Float.valueOf((float) value);
        } else if (type == Integer.class) {
            return (V) Integer.valueOf((int) value);
        } else if (type == Long.class) {
            return (V) Long.valueOf((long) value);
        } else if (type == Short.class) {
            return (V) Short.valueOf((short) value);
        } else if (type == Byte.class) {
            return (V) Byte.valueOf((byte) value);
        }
        throw new IllegalArgumentException("Unsupported numeric type: " + type);
    }
}

package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import java.util.Comparator;

public class InterpolatingTreeMapTest {

    private static final double DELTA = 1e-9;

    /**
     * Creates a standard InterpolatingTreeMap for Double keys and values.
     */
    private InterpolatingTreeMap<Double, Double> createDoubleMap() {
        return new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            Interpolator.forDouble()
        );
    }

    @Test
    void get_exactKeyMatch_returnsExactValue() {
        InterpolatingTreeMap<Double, Double> map = createDoubleMap();
        map.put(1.0, 10.0);
        map.put(2.0, 20.0);
        map.put(3.0, 30.0);

        assertEquals(10.0, map.get(1.0), DELTA);
        assertEquals(20.0, map.get(2.0), DELTA);
        assertEquals(30.0, map.get(3.0), DELTA);
    }

    @Test
    void get_keyBetweenTwoEntries_interpolatesLinearly() {
        InterpolatingTreeMap<Double, Double> map = createDoubleMap();
        map.put(0.0, 0.0);
        map.put(10.0, 100.0);

        // Key 5.0 is halfway between 0.0 and 10.0, so value should be 50.0
        assertEquals(50.0, map.get(5.0), DELTA);

        // Key 2.5 is 25% of the way, so value should be 25.0
        assertEquals(25.0, map.get(2.5), DELTA);

        // Key 7.5 is 75% of the way, so value should be 75.0
        assertEquals(75.0, map.get(7.5), DELTA);
    }

    @Test
    void get_keyBetweenNonZeroEntries_interpolatesCorrectly() {
        InterpolatingTreeMap<Double, Double> map = createDoubleMap();
        map.put(10.0, 100.0);
        map.put(20.0, 200.0);

        // Key 15.0 is halfway between 10.0 and 20.0
        assertEquals(150.0, map.get(15.0), DELTA);

        // Key 12.0 is 20% of the way
        assertEquals(120.0, map.get(12.0), DELTA);
    }

    @Test
    void get_keyBelowAllEntries_returnsLowestValue() {
        InterpolatingTreeMap<Double, Double> map = createDoubleMap();
        map.put(5.0, 50.0);
        map.put(10.0, 100.0);

        // Key 0.0 is below all entries, should return the floor key's value
        assertEquals(50.0, map.get(0.0), DELTA);
        assertEquals(50.0, map.get(-10.0), DELTA);
    }

    @Test
    void get_keyAboveAllEntries_returnsHighestValue() {
        InterpolatingTreeMap<Double, Double> map = createDoubleMap();
        map.put(5.0, 50.0);
        map.put(10.0, 100.0);

        // Key 15.0 is above all entries, should return the ceiling key's value
        assertEquals(100.0, map.get(15.0), DELTA);
        assertEquals(100.0, map.get(100.0), DELTA);
    }

    @Test
    void get_emptyMap_returnsNull() {
        InterpolatingTreeMap<Double, Double> map = createDoubleMap();

        assertNull(map.get(5.0));
    }

    @Test
    void get_singleEntry_returnsThatValue() {
        InterpolatingTreeMap<Double, Double> map = createDoubleMap();
        map.put(5.0, 50.0);

        // Any key should return the single value
        assertEquals(50.0, map.get(5.0), DELTA);
        assertEquals(50.0, map.get(0.0), DELTA);
        assertEquals(50.0, map.get(100.0), DELTA);
    }

    @Test
    void clear_removesAllEntries() {
        InterpolatingTreeMap<Double, Double> map = createDoubleMap();
        map.put(1.0, 10.0);
        map.put(2.0, 20.0);

        map.clear();

        assertNull(map.get(1.0));
        assertNull(map.get(2.0));
    }

    @Test
    void put_duplicateKey_overwritesValue() {
        InterpolatingTreeMap<Double, Double> map = createDoubleMap();
        map.put(5.0, 50.0);
        map.put(5.0, 100.0);

        assertEquals(100.0, map.get(5.0), DELTA);
    }

    @Test
    void get_multipleIntermediateKeys_interpolatesCorrectly() {
        InterpolatingTreeMap<Double, Double> map = createDoubleMap();
        map.put(0.0, 0.0);
        map.put(10.0, 100.0);
        map.put(20.0, 400.0);

        // Between 0 and 10: linear interpolation
        assertEquals(50.0, map.get(5.0), DELTA);

        // Between 10 and 20: linear interpolation (100 to 400, range of 300)
        // At 15.0: 50% of the way, so 100 + 150 = 250
        assertEquals(250.0, map.get(15.0), DELTA);

        // At 12.0: 20% of the way between 10 and 20
        // 100 + 0.2 * 300 = 160
        assertEquals(160.0, map.get(12.0), DELTA);
    }

    @Test
    void constructor_withComparator_usesComparator() {
        // Create map with reverse comparator
        InterpolatingTreeMap<Double, Double> map = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            Interpolator.forDouble(),
            Comparator.reverseOrder()
        );

        map.put(10.0, 100.0);
        map.put(20.0, 200.0);

        // Exact key lookup should still work
        assertEquals(100.0, map.get(10.0), DELTA);
        assertEquals(200.0, map.get(20.0), DELTA);

        // With reverse order, 20.0 is "lower" than 10.0 in tree ordering
        // Key 25.0 is below all entries (since 25 > 20 > 10 means 25 is "smallest" in reverse order)
        // So it should return the value of the floor key (20.0 in reverse order), which is 200.0
        assertEquals(200.0, map.get(25.0), DELTA);

        // Key 5.0 is above all entries in reverse order (5 < 10 < 20 means 5 is "largest")
        // So it should return the ceiling key's value (10.0), which is 100.0
        assertEquals(100.0, map.get(5.0), DELTA);
    }

    @Test
    void get_negativeValues_interpolatesCorrectly() {
        InterpolatingTreeMap<Double, Double> map = createDoubleMap();
        map.put(-10.0, -100.0);
        map.put(10.0, 100.0);

        // Key 0.0 is halfway between -10.0 and 10.0
        assertEquals(0.0, map.get(0.0), DELTA);

        // Key -5.0 is 25% of the way
        assertEquals(-50.0, map.get(-5.0), DELTA);
    }

    @Test
    void get_decreasingValues_interpolatesCorrectly() {
        InterpolatingTreeMap<Double, Double> map = createDoubleMap();
        map.put(0.0, 100.0);
        map.put(10.0, 0.0);

        // Key 5.0 is halfway, value should be 50.0
        assertEquals(50.0, map.get(5.0), DELTA);

        // Key 2.5 is 25% of the way, value should be 75.0
        assertEquals(75.0, map.get(2.5), DELTA);
    }

    @Test
    void customInterpolator_worksCorrectly() {
        // Custom interpolator that does quadratic interpolation
        Interpolator<Double> quadraticInterpolator = (start, end, t) -> {
            double tClamped = MathUtil.clamp(t, 0, 1);
            return start + (end - start) * tClamped * tClamped;
        };

        InterpolatingTreeMap<Double, Double> map = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            quadraticInterpolator
        );

        map.put(0.0, 0.0);
        map.put(10.0, 100.0);

        // At t=0.5, quadratic gives 0.25, so value = 0 + 100 * 0.25 = 25
        assertEquals(25.0, map.get(5.0), DELTA);
    }
}

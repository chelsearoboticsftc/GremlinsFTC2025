package org.firstinspires.ftc.teamcode.utils;
import org.firstinspires.ftc.teamcode.utils.LookupTable.Pair;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

public class LookupTableTest {
    @Test public void testEqual() {
        List<Pair> pairs = new ArrayList<>();
        for (int i = 0; i <= 10; i++) {
            pairs.add(new Pair(i, i));
        }
        LookupTable t = new LookupTable(pairs);
        // because our pairs are all input = output, we should get back the input
        assertEquals(4.2, t.interpolate(4.2), 1e-9);
    }

    @Test public void test2x() {
        List<Pair> pairs = new ArrayList<>();
        for (int i = 0; i <= 10; i++) {
            pairs.add(new Pair(i, 2 * i));
        }
        LookupTable t = new LookupTable(pairs);
        // we should get back 2 * x for any input value
        for (double x : new double[] {0, 5.8, 3.14, 2.68}) {
            assertEquals(2 * x, t.interpolate(x), 1e-9);
        }
    }

    @Test public void testClamping() {
        List<Pair> pairs = new ArrayList<>();
        for (int i = 0; i <= 10; i++) {
            pairs.add(new Pair(i, i));
        }
        LookupTable t = new LookupTable(pairs);
        // looking up values beyond the provided range should be clamped
        assertEquals(0.0, t.interpolate(-10.0), 1e-9);
        assertEquals(10, t.interpolate(20.0), 1e-9);
    }
}
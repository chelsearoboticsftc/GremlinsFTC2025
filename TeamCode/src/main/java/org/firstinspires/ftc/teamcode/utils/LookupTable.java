package org.firstinspires.ftc.teamcode.utils;

import java.util.*;

/**
 * LookupTable performs linear interpolation over user-supplied (input, output) points.
 * - Points are sorted by input value on construction.
 * - Duplicate inputs values are rejected.
 * - For inputs below/above the table, it clamps to the given input range
 */
public final class LookupTable {
    public static final class Pair {
        public final double input;
        public final double output;
        public Pair(double input, double output) { this.input = input; this.output = output; }
    }

    private final double[] inputs; // strictly increasing
    private final double[] outputs; // corresponding output values

    /**
     * @param points list of (input, output) pairs; must contain at least 2 distinct input values
     */
    public LookupTable(List<Pair> points) {
        if (points == null || points.size() < 2)
            throw new IllegalArgumentException("At least two points are required.");

        // Copy and sort by x
        List<Pair> sorted = new ArrayList<>(points);
        sorted.sort(Comparator.comparingDouble(p -> p.input));

        // Validate strictly increasing x and load arrays
        inputs = new double[sorted.size()];
        outputs = new double[sorted.size()];
        inputs[0] = sorted.get(0).input;
        outputs[0] = sorted.get(0).output;
        for (int i = 1; i < sorted.size(); i++) {
            double x = sorted.get(i).input;
            double y = sorted.get(i).output;
            if (x == inputs[i - 1]) {
                throw new IllegalArgumentException("Duplicate x value: " + x);
            }
            inputs[i] = x;
            outputs[i] = y;
        }
    }

    private int closestIndex(double input) {
        // https://developer.android.com/reference/java/util/Arrays#binarySearch(byte[],%20int,%20int,%20byte)
        int idx = Arrays.binarySearch(inputs, input);
        if (idx >= 0) return idx;
        else return -idx - 1;
    }

    /**
     * Finds the closest output value for the given input.
     */

    public double closestValue(double input) {
        return outputs[closestIndex(input)];
    }

    /**
     * Interpolates (or extrapolates) output for the given input.
     */

    public double interpolate(double input) {
        int index = closestIndex(input);

        int n = inputs.length;
        // Exact matches on boundaries or within
        if (index == 0) {
            // input is below the smallest value - use first output
            return outputs[0];
        } else if (index == n) {
            // input is above the largest value - use last output
            return outputs[n - 1];
        } else {
            // interpolate between the surrounding points
            double x0 = inputs[index - 1];
            double y0 = outputs[index - 1];
            double x1 = inputs[index];
            double y1 = outputs[index];
            double deltaX = x1 - x0;
            double deltaY = y1 - y0;
            double fraction = (input - x0) / deltaX;
            return y0 + fraction * deltaY;
        }
    }

    public int size() { return inputs.length; }
    public double minInput() { return inputs[0]; }
    public double maxInput() { return inputs[inputs.length - 1]; }

    // Convenience builder from primitive arrays (must be same length)
    public static LookupTable fromArrays(double[] inputs, double[] outputs) {
        if (inputs == null || outputs == null || inputs.length != outputs.length)
            throw new IllegalArgumentException("inputs and outputs must be non-null and the same length.");
        List<Pair> pts = new ArrayList<>(inputs.length);
        for (int i = 0; i < inputs.length; i++) pts.add(new Pair(inputs[i], outputs[i]));
        return new LookupTable(pts);
    }
}

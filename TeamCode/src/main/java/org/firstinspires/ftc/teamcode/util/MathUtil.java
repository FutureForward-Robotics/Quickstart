// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.util;

public final class MathUtil {

    private MathUtil() {}

    public static int clamp(int value, int low, int high) {
        return Math.max(low, Math.min(value, high));
    }

    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    /** Linear interpolation. {@code t} is clamped to [0, 1]. */
    public static double interpolate(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * clamp(t, 0, 1);
    }

    /** Where {@code q} falls in [0, 1] between the two values. Returns 0 for an empty range. */
    public static double inverseInterpolate(double startValue, double endValue, double q) {
        double totalRange = endValue - startValue;
        if (totalRange <= 0) {
            return 0.0;
        }
        double queryToStart = q - startValue;
        if (queryToStart <= 0) {
            return 0.0;
        }
        return queryToStart / totalRange;
    }
}

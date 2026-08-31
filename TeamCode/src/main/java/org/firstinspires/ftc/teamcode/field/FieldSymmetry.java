package org.firstinspires.ftc.teamcode.field;

/**
 * How the two alliance halves of the field relate. Set {@link Field#SYMMETRY} to the one this
 * season's field uses.
 *
 * <p>Reflections invert handedness: a path that curves left on red curves right on blue. Rotation
 * preserves it. All three are involutions, so applying one twice returns the original pose.
 */
public enum FieldSymmetry {

    /** Reflect across the vertical centre line. */
    MIRROR_X {
        @Override
        public double x(double x, double y) {
            return Field.WIDTH_IN - x;
        }

        @Override
        public double y(double x, double y) {
            return y;
        }

        @Override
        public double heading(double headingRad) {
            return Field.normalize(Math.PI - headingRad);
        }
    },

    /** Reflect across the horizontal centre line. */
    MIRROR_Y {
        @Override
        public double x(double x, double y) {
            return x;
        }

        @Override
        public double y(double x, double y) {
            return Field.WIDTH_IN - y;
        }

        @Override
        public double heading(double headingRad) {
            return Field.normalize(-headingRad);
        }
    },

    /** Rotate 180 degrees about the field centre. */
    ROTATIONAL {
        @Override
        public double x(double x, double y) {
            return Field.WIDTH_IN - x;
        }

        @Override
        public double y(double x, double y) {
            return Field.WIDTH_IN - y;
        }

        @Override
        public double heading(double headingRad) {
            return Field.normalize(headingRad + Math.PI);
        }
    };

    public abstract double x(double x, double y);

    public abstract double y(double x, double y);

    public abstract double heading(double headingRad);
}

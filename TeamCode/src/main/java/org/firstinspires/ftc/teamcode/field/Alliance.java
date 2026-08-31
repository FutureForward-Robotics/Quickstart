package org.firstinspires.ftc.teamcode.field;

/** Alliance colour. Pass as a value; do not hold it in a mutable static. */
public enum Alliance {
    RED,
    BLUE;

    public boolean isRed() {
        return this == RED;
    }

    public Alliance other() {
        return this == RED ? BLUE : RED;
    }
}

package org.firstinspires.ftc.teamcode.field;

/** Alliance colour. */
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

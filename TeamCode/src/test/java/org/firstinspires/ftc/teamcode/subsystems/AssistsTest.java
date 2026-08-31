package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Waypoint;
import org.junit.jupiter.api.Test;

/**
 * Assists are pure functions, so the interesting behaviour is tested on a laptop. The Drive
 * subsystem itself needs a Follower and is deliberately thin glue.
 */
class AssistsTest {

    private static final double EPS = 1e-9;
    private static final DriveInput STRAIGHT = new DriveInput(0.6, 0.0, 0.0);

    @Test
    void headingLockCorrectsTowardTarget() {
        DriveAssist lock = Assists.headingLock(0, 2.0);
        Pose off = new Pose(0, 0, Math.toRadians(-20));

        DriveInput out = lock.apply(STRAIGHT, off);

        assertTrue(out.turn > 0, "robot is 20deg clockwise of target, so it must turn CCW");
        assertEquals(0.6, out.forward, EPS, "assists must not touch translation here");
    }

    @Test
    void headingLockYieldsWhileTheDriverIsTurning() {
        DriveAssist lock = Assists.headingLock(0, 2.0);
        Pose off = new Pose(0, 0, Math.toRadians(-20));
        DriveInput driverTurning = new DriveInput(0.6, 0, -0.8);

        assertEquals(-0.8, lock.apply(driverTurning, off).turn, EPS, "driver always wins");
    }

    @Test
    void headingLockOutputIsClamped() {
        DriveAssist lock = Assists.headingLock(0, 50.0);
        DriveInput out = lock.apply(STRAIGHT, new Pose(0, 0, Math.toRadians(-170)));
        assertTrue(Math.abs(out.turn) <= 1.0, "must never command beyond full power");
    }

    @Test
    void headingLockTakesTheShortWayAround() {
        DriveAssist lock = Assists.headingLock(-175, 1.0);
        // Robot at +175deg. The short path is +10deg (CCW), not -350deg.
        DriveInput out = lock.apply(STRAIGHT, new Pose(0, 0, Math.toRadians(175)));
        assertTrue(out.turn > 0, "wrap must be handled, otherwise the robot spins the long way");
        assertEquals(Math.toRadians(10), out.turn, 1e-6);
    }

    @Test
    void slowNearScalesTranslationInsideTheRadius() {
        Waypoint goal = Waypoint.red("goal", 100, 100, 0);
        DriveAssist slow = Assists.slowNear(goal, Alliance.RED, 20.0, 0.25);

        assertEquals(0.6, slow.apply(STRAIGHT, new Pose(100, 60, 0)).forward, EPS, "outside radius");
        assertEquals(0.6 * 0.25, slow.apply(STRAIGHT, new Pose(100, 100, 0)).forward, 1e-9, "at goal");

        double halfway = slow.apply(STRAIGHT, new Pose(100, 90, 0)).forward;
        assertEquals(0.6 * (0.25 + 0.75 * 0.5), halfway, 1e-9);
    }

    @Test
    void slowNearLeavesTurnAlone() {
        Waypoint goal = Waypoint.red("goal", 100, 100, 0);
        DriveAssist slow = Assists.slowNear(goal, Alliance.RED, 20.0, 0.25);
        DriveInput turning = new DriveInput(0.6, 0, 0.9);
        assertEquals(0.9, slow.apply(turning, new Pose(100, 100, 0)).turn, EPS);
    }

    @Test
    void slowNearFollowsTheAllianceMirror() {
        Waypoint goal = Waypoint.red("goal", 100, 100, 0);
        DriveAssist blue = Assists.slowNear(goal, Alliance.BLUE, 20.0, 0.25);
        // Mirrored goal is x=44, so the red-side position is now far away and unscaled.
        assertEquals(0.6, blue.apply(STRAIGHT, new Pose(100, 100, 0)).forward, EPS);
        assertEquals(0.6 * 0.25, blue.apply(STRAIGHT, new Pose(44, 100, 0)).forward, 1e-9);
    }

    @Test
    void pullTowardIsCappedAndOnlyActsInsideTheRadius() {
        Waypoint goal = Waypoint.red("goal", 100, 100, 0);
        DriveAssist pull = Assists.pullToward(goal, Alliance.RED, 1.0, 20.0, 0.3);

        assertEquals(0.6, pull.apply(STRAIGHT, new Pose(0, 0, 0)).forward, EPS, "far away, untouched");

        DriveInput near = pull.apply(STRAIGHT, new Pose(90, 100, 0));
        assertEquals(0.6 + 0.3, near.forward, EPS, "10in error at kP=1 clamps to maxAuthority");
        assertEquals(0.0, near.turn, EPS);
    }

    @Test
    void assistsCompose() {
        Waypoint goal = Waypoint.red("goal", 100, 100, 0);
        DriveInput out =
                Assists.slowNear(goal, Alliance.RED, 20.0, 0.5)
                        .apply(
                                Assists.headingLock(0, 2.0).apply(STRAIGHT, new Pose(100, 100, 0)),
                                new Pose(100, 100, 0));

        assertEquals(0.3, out.forward, EPS, "translation halved by slowNear");
        assertEquals(0.0, out.turn, EPS, "already on heading");
    }

    @Test
    void speedCapScalesTranslationOnly() {
        DriveInput out = Assists.speedCap(0.4).apply(new DriveInput(1.0, 1.0, 1.0), new Pose());
        assertEquals(0.4, out.forward, EPS);
        assertEquals(0.4, out.strafe, EPS);
        assertEquals(1.0, out.turn, EPS);
    }

    @Test
    void driveInputIsImmutable() {
        DriveInput original = new DriveInput(1, 2, 3);
        original.withTurn(9).scaledTranslation(0.5).plus(1, 1, 1);
        assertEquals(1, original.forward, EPS);
        assertEquals(2, original.strafe, EPS);
        assertEquals(3, original.turn, EPS);
    }
}

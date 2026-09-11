package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Waypoint;
import org.junit.jupiter.api.Test;

/** Assists are pure functions. Drive needs a Follower and is not covered here. */
class AssistsTest {

    private static final double EPS = 1e-9;
    private static final DriveInput STRAIGHT = new DriveInput(0.6, 0.0, 0.0);

    @Test
    void headingLockCorrectsTowardTarget() {
        DriveAssist lock = Assists.headingLock(0, 2.0);
        Pose off = new Pose(0, 0, Math.toRadians(-20));

        DriveInput out = lock.apply(STRAIGHT, off);

        assertTrue(out.turn > 0, "robot is 20deg clockwise of target, so it must turn CCW");
        assertEquals(0.6, out.forward, EPS);
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
        assertTrue(Math.abs(out.turn) <= 1.0);
    }

    @Test
    void headingLockTakesTheShortWayAround() {
        DriveAssist lock = Assists.headingLock(-175, 1.0);
        // Robot at +175deg; short path is +10deg, not -350deg.
        DriveInput out = lock.apply(STRAIGHT, new Pose(0, 0, Math.toRadians(175)));
        assertTrue(out.turn > 0, "must not spin the long way");
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
        // Mirrored goal is x=44.
        assertEquals(0.6, blue.apply(STRAIGHT, new Pose(100, 100, 0)).forward, EPS);
        assertEquals(0.6 * 0.25, blue.apply(STRAIGHT, new Pose(44, 100, 0)).forward, 1e-9);
    }

    @Test
    void pullTowardIsCappedAndOnlyActsInsideTheRadius() {
        Waypoint goal = Waypoint.red("goal", 100, 100, 0);
        DriveAssist pull = Assists.pullToward(goal, Alliance.RED, 1.0, 20.0, 0.3);

        assertEquals(0.6, pull.apply(STRAIGHT, new Pose(0, 0, 0)).forward, EPS);

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

        assertEquals(0.3, out.forward, EPS);
        assertEquals(0.0, out.turn, EPS);
    }

    @Test
    void speedCapScalesTranslationOnly() {
        DriveInput out = Assists.speedCap(0.4).apply(new DriveInput(1.0, 1.0, 1.0), Pose.zero());
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

    @Test
    void steadyShotHoldsOneSpeedHoweverHardTheStickIsPushed() {
        DriveAssist armed = Assists.steadyShot(() -> 0, 2.0, 0.35);
        Pose square = new Pose(0, 0, 0);

        DriveInput gentle = armed.apply(new DriveInput(0.2, 0, 0), square);
        DriveInput full = armed.apply(new DriveInput(1.0, 0, 0), square);

        assertEquals(0.35, Math.hypot(gentle.forward, gentle.strafe), EPS);
        assertEquals(0.35, Math.hypot(full.forward, full.strafe), EPS);
    }

    @Test
    void steadyShotKeepsTheDirectionTheStickAsksFor() {
        DriveAssist armed = Assists.steadyShot(() -> 0, 2.0, 0.5);

        DriveInput out = armed.apply(new DriveInput(0.3, 0.3, 0), new Pose(0, 0, 0));

        assertEquals(0.5, Math.hypot(out.forward, out.strafe), EPS);
        assertEquals(out.forward, out.strafe, EPS, "45deg in stays 45deg out");
    }

    @Test
    void steadyShotStandsStillWhenTheStickIsCentred() {
        DriveAssist armed = Assists.steadyShot(() -> 0, 2.0, 0.35);

        DriveInput out = armed.apply(new DriveInput(0.02, -0.01, 0), new Pose(0, 0, 0));

        assertEquals(0, out.forward, EPS);
        assertEquals(0, out.strafe, EPS);
    }

    @Test
    void steadyShotHoldsTheHeadingEvenAgainstTheDriver() {
        DriveAssist armed = Assists.steadyShot(() -> 0, 2.0, 0.35);
        Pose off = new Pose(0, 0, Math.toRadians(-20));

        DriveInput out = armed.apply(new DriveInput(0.6, 0, -0.9), off);

        assertTrue(out.turn > 0, "corrects CCW toward the held heading, ignoring the stick");
        assertTrue(Math.abs(out.turn) <= 1.0, "turn output is clamped");
    }
}

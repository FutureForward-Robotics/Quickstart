package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.ForwardOpMode;

/**
 * Match teleop. Subclasses supply only the alliance, so the two sides cannot drift apart.
 *
 * <p>{@code configure()} is final; put shared setup here and per-alliance values in the subclass.
 */
public abstract class MatchTeleOp extends ForwardOpMode {

    protected Robot robot;

    protected abstract Alliance alliance();

    @Override
    protected final void configure() {
        robot = new Robot(hardwareMap, alliance());
        DriverBindings.configure(robot, driver, operator);
    }

    @Override
    public void run() {
        super.run();
        if (telemetryDue(250)) {
            telemetry.addData("alliance", robot.alliance);
            telemetry.addData("loop", "%.0f Hz", loopHz());
            telemetry.addData("pose", robot.drive.pose());
            telemetry.update();
        }
    }
}

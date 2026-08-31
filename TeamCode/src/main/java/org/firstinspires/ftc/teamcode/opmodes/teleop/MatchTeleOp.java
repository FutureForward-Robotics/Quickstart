package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.ForwardOpMode;

/**
 * Match teleop. Subclasses supply the alliance; {@code configure()} is final and does the rest.
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

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;

/**
 * The tuners AutoTune offers. Each method must be static, take no arguments and return a {@link
 * Procedure}; a scanner finds them when the app loads, so there is nothing to register.
 *
 * <p>The Robot Controller serves the tuning page on port 10158 for as long as the app is running:
 * on a Control Hub, {@code http://192.168.43.1:10158}. There is no OpMode to select; the page
 * starts the OpModes each tuner needs.
 *
 * <p>Run them in this order: the drivetrain before the localizer, and both before Foresight, which
 * drives the robot and needs the other two working. Each tuner prints a block of code to paste into
 * {@link Constants}.
 *
 * <p>Only the tuners for the hardware this template wires are here. For OTOS, OctoQuad, two- or
 * three-wheel odometry, or swerve, copy the matching procedure from Pedro's Quickstart into {@code
 * pedroPathing/procedures} and add a method for it.
 */
public final class Tuning {

    private Tuning() {}

    @Tuner(name = "Mecanum Drivetrain")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint Localizer")
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "Foresight")
    public static Procedure foresight() {
        return new ForesightTuner(Constants::localizer, Constants::drivetrain);
    }

    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(Constants::drivetrain, Constants::localizer, Constants::algorithm);
    }
}

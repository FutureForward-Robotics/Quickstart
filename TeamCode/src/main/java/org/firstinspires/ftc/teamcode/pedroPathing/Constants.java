package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing configuration: a mecanum drivetrain, a Pinpoint localizer, and the Foresight
 * following algorithm.
 *
 * <p>Every number here comes from AutoTune. Run the tuners from the Driver Station, open the page
 * it hosts, and paste the block each one prints over the matching config below. The values checked
 * in are placeholders that let the code compile; they do not describe any real robot.
 *
 * <p>Foresight requires the velocity, deceleration and braking values: it plans a braking point
 * from them rather than easing toward the end, so a wrong number shows up as overshoot.
 */
public class Constants {

    public static final MecanumConfig drivetrainConfig =
            new MecanumConfig(
                    c -> {
                        c.frontLeftName.set("leftFront");
                        c.frontRightName.set("rightFront");
                        c.backLeftName.set("leftRear");
                        c.backRightName.set("rightRear");
                        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                    });

    public static final PinpointConfig localizerConfig =
            new PinpointConfig(
                    c -> {
                        c.name.set("pinpoint");
                        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                        c.xPodOffset.set(0.0);
                        c.yPodOffset.set(0.0);
                        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                        c.globalDistanceUnit.set(DistanceUnit.INCH);
                        c.offsetUnits.set(DistanceUnit.INCH);
                    });

    public static final ForesightConfig algorithmConfig =
            new ForesightConfig(
                    c -> {
                        c.forwardTranslational.set(Controller.proportional(0.1));
                        c.strafeTranslational.set(Controller.proportional(0.1));

                        c.coast.set(Controller.proportionalFeedforward(0.01));
                        c.brake.set(Controller.proportionalFeedforward(0.01));

                        c.headingFeedback.set(Controller.proportional(1.0));
                        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.1, 0.1));

                        c.linearBrakeCoefficients.set(Matrix.diag(0.1, 0.1));
                        c.quadraticBrakeCoefficients.set(Matrix.diag(0.01, 0.01));

                        c.maxAchievableForwardVelocity.set(60.0);
                        c.maxAchievableStrafeVelocity.set(50.0);
                        c.naturalForwardDeceleration.set(-30.0);
                        c.naturalStrafeDeceleration.set(-40.0);
                    });

    /** Hardware factories, separate so the AutoTune procedures can build them on their own. */
    public static Localizer localizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Drivetrain drivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Algorithm algorithm() {
        return new Foresight(algorithmConfig);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(localizer(hardwareMap), drivetrain(hardwareMap), algorithm());
    }
}

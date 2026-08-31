package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.Alliance;

@TeleOp(name = "TeleOp Blue", group = "match")
public class TeleOpBlue extends MatchTeleOp {

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}

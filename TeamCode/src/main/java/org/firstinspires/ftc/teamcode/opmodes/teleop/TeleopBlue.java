package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class TeleopBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TeleopCommon teleop = new TeleopCommon();
        teleop.setTagID(20);
        teleop.runOpMode();

    }
}

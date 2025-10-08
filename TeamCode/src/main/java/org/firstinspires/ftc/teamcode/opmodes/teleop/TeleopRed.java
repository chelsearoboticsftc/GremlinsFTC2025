package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp
public class TeleopRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TeleopCommon teleop = new TeleopCommon();
        teleop.setTagID(24);
        teleop.runOpMode();

    }
}

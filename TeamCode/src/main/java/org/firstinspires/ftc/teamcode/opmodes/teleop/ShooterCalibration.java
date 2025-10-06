package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
public class ShooterCalibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "leftFront");
        waitForStart();

        float step = 0.1F;
        float speed = 0F;

        while(opModeIsActive()){
            this.telemetry.addData("Speed", speed);
            this.telemetry.update();

            if (gamepad1.aWasPressed()) {
                speed += step;
            }
            motor.setPower(speed);
        }
    }
}

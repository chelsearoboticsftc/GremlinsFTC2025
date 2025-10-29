package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp
public class TestForKirk extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        DcMotorEx shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        Servo upperLeftGate = hardwareMap.get(Servo.class, "upperLeftGate");
        Servo upperRightGate = hardwareMap.get(Servo.class, "upperRightGate");

        boolean shooterOn = false;
        upperLeftGate.setPosition(1);
        upperRightGate.setPosition(0.0);
        waitForStart();

        while(opModeIsActive()) {

            double speedFactor = 1 - gamepad1.right_trigger + 0.2;
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y * speedFactor,
                                    -gamepad1.left_stick_x * speedFactor
                            ),
                            -gamepad1.right_stick_x
                    )
            );
            if (this.gamepad2.y) {
//                shooterRight.setPower(-1);
                shooterLeft.setPower(-1);
            }
            else {
                if (this.gamepad2.bWasPressed()) {
                    shooterOn = !shooterOn;
                }
                if (shooterOn) {
//                    shooterRight.setPower(1);
                    shooterLeft.setPower(1);
                } else {
//                    shooterRight.setPower(0);
                    shooterLeft.setPower(0);
                }
            }


            if (this.gamepad2.a)
                intake.setPower(-1);
            else

                if (this.gamepad2.x)
                    intake.setPower(1);
                else
                    intake.setPower(0);


            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                upperLeftGate.setPosition(0.5);
            }

            if ((gamepad2.left_trigger > 0.1) || (gamepad2.right_trigger > 0.1)) {
                upperLeftGate.setPosition(1);
            }

            if (gamepad2.right_bumper || gamepad2.left_bumper) {
                upperRightGate.setPosition(0.4);
            }

            if ((gamepad2.right_trigger > 0.1) || (gamepad2.left_trigger > 0.1)){
                upperRightGate.setPosition(0.0);
            }

            this.telemetry.addData("left Servo", upperLeftGate.getPosition());
            this.telemetry.addData("right Servo", upperRightGate.getPosition());
            this.telemetry.update();

        }

    }
}

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.example.SmartShooter;


@TeleOp
public class TestForKirk2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        DcMotorEx intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        DcMotorEx intake1 = hardwareMap.get(DcMotorEx.class, "intake");
        SmartShooter shooter = new SmartShooter(hardwareMap);
        boolean intakeon = false;
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

            if (this.gamepad2.aWasPressed())
                intakeon = !intakeon;


            if (this.gamepad2.x) {
                intake1.setPower(1);
                intake2.setPower(1);
            }
            else {
                if (intakeon) {
                    intake1.setPower(-1);
                    intake2.setPower(-1);
                    shooter.setMotorVelocity(350);
                }
                else {
                    intake1.setPower(0);
                    intake2.setPower(0);
                    shooter.setPower(0);
                }
            }

            if (gamepad2.left_bumper)
                shooter.raiseLeftGate();

            if (gamepad2.left_trigger > 0.1)
                shooter.lowerLeftGate();

            if(gamepad2.right_bumper)
                shooter.raiseRightGate();

            if (gamepad2.right_trigger > 0.1)
                shooter.lowerRightGate();

        }

    }
}

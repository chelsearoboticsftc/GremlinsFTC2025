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
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        SmartShooter shooter = new SmartShooter(hardwareMap);
        boolean shooterOn = false;
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
                shooter.setPower(-0.9);
            }
            else {
                if (this.gamepad2.bWasPressed()) {
                    shooterOn = !shooterOn;
                }
                if (shooterOn) {
                    shooter.setPower(0.9);
                } else {
                    shooter.setPower(0);
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
                shooter.raiseGates();
            }

            if ((gamepad2.left_trigger > 0.1) || (gamepad2.right_trigger > 0.1)) {
                shooter.lowerGates();
            }
        }

    }
}

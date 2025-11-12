package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.example.Intake;
import org.firstinspires.ftc.teamcode.subsystems.example.SmartShooter;


public class TeleopCommon extends LinearOpMode {
    int tagID = 20;

    public void setTagID(int tagID) {
        this.tagID = tagID;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        SmartShooter shooter = new SmartShooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        waitForStart();


        while(opModeIsActive()){
            double speedFactor = 1 - gamepad1.right_trigger + 0.2;
            drive.setDrivePowers(
                new PoseVelocity2d(
                    new Vector2d(
                        gamepad1.left_stick_y * speedFactor,
                        gamepad1.left_stick_x * speedFactor),
                    gamepad1.right_stick_x
                    )
                );
            if(gamepad2.right_bumper){
                shooter.shoot(10, false, true);
            }
            if(gamepad2.left_bumper){
                shooter.shoot(10, true, false);
            }


        }
    }
}

package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.example.Intake;
import org.firstinspires.ftc.teamcode.subsystems.example.SmartShooter;

@Autonomous
public class BlueNearAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize MecanumDrive. The starting pose is (0, 0) with a 0-degree heading.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        SmartShooter shooter = new SmartShooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        // Wait for the driver to press start
        waitForStart();
        intake.setMotorPower(0.5);
        shooter.shoot(10);
        Thread.sleep(10000);
        double setMotorVelocity = 0;
        if (isStopRequested()) return;

        // Road Runner uses inches, so we convert 10 feet to 120 inches.
        // We create an "action" to drive forward (along the X-axis) by 120 inches.
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(12)
                        .build()
        );
    }
}
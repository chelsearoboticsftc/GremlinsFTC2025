package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        // Wait for the driver to press start
        waitForStart();

        intake.setVelocity(-1500);
        intake2.setVelocity(-1500);
        shooter.setMotorVelocity(350);
        Thread.sleep(5000);
        shooter.raiseLeftGate();
        Thread.sleep(2000);
        shooter.lowerLeftGate();
        shooter.raiseRightGate();
        Thread.sleep(2000);
        shooter.lowerRightGate();
        shooter.setPower(0);
        intake.setPower(0);
        intake2.setPower(0);

        if (isStopRequested()) return;

        // Road Runner uses inches, so we convert 10 feet to 120 inches.
        // We create an "action" to drive forward (along the X-axis) by 120 inches.
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(24)
                        .turn(-0.5)
                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(36)
                        .build()
        );

    }
}
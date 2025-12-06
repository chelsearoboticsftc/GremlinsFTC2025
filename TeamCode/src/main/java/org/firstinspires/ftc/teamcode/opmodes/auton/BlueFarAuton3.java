package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.example.SmartShooter;


@Disabled
@Autonomous
public class BlueFarAuton3 extends LinearOpMode {
    // grid size is handy for describing distances
    final double gridSize = 23.5;
    // starting position - backed up to goal, ready to shoot
    final Pose2d startingPos = new Pose2d(0.7 * gridSize, -2.667 * gridSize, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPos);
        drive.localizer.setPose(startingPos);
        SmartShooter shooter = new SmartShooter(hardwareMap);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        // Wait for the driver to press start
        waitForStart();

        // start intake and flywheel
        shooter.setMotorVelocity(350);
        Thread.sleep(1500);
        intake.setVelocity(-1575);
        intake2.setVelocity(-1575);
        Thread.sleep(1000);

        // load field artifacts
        Actions.runBlocking(firstPart(drive));
        shoot(shooter);
        Actions.runBlocking(secondPart(drive));
        shoot(shooter);
        Actions.runBlocking(thirdPart(drive));
        shoot(shooter);


        // leave line
        Actions.runBlocking(
                drive.actionBuilder(startingPos)
                        .splineTo(new Vector2d(-1.5 * gridSize, 0), Math.toRadians(-90))
                        .build()
        );

        // turn off intake and flywheel
        shooter.setPower(0);
        intake.setPower(0);
    }

    private Action firstPart(MecanumDrive drive) {
        return drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(new Pose2d(-2 * gridSize, 2.3 * gridSize, Math.toRadians(-37)), Math.toRadians(180))
                .build();
    }
    private Action secondPart(MecanumDrive drive) {
        return drive.actionBuilder(drive.localizer.getPose())
                .setTangent(Math.toRadians(180))
                .lineToX(-0.5 * gridSize)
                .turnTo(Math.toRadians(-90))
                .lineToY(-1.5 * gridSize)
                .turnTo(Math.toRadians(180))
                .lineToX(-2.5 * gridSize)
                .turnTo(Math.toRadians(180))
                .lineToX(-1.5 * gridSize)
                .splineToLinearHeading(new Pose2d(-2 * gridSize, 2.3 * gridSize, Math.toRadians(-37)), Math.toRadians(180))
                .build();
    }

    private Action thirdPart(MecanumDrive drive) {
        return drive.actionBuilder(drive.localizer.getPose())
                .setTangent(Math.toRadians(180))
                .lineToX(-0.5 * gridSize)
                .turnTo(Math.toRadians(-90))
                .lineToY(-1.5 * gridSize)
                .build();
    }

    private void shoot(SmartShooter shooter) throws InterruptedException {
        shooter.raiseLeftGate();
        Thread.sleep(1500);
        shooter.raiseRightGate();
        Thread.sleep(1500);
        shooter.lowerGates();
    }
}


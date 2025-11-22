package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.example.SmartShooter;


@Autonomous
public class RedNearAuton2 extends LinearOpMode {
    // grid size is handy for describing distances
    final double gridSize = 23.5;
    // starting position - backed up to goal, ready to shoot
    final Pose2d startingPos = new Pose2d(2.1 * gridSize, 2.3 * gridSize, Math.toRadians(225));

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPos);
        SmartShooter shooter = new SmartShooter(hardwareMap);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Wait for the driver to press start
        waitForStart();

        // start intake and flywheel
        shooter.setMotorVelocity(350);
        Thread.sleep(1000);
        intake.setPower(-1);
        Thread.sleep(1000);

        // shoot preloaded artifacts
        shoot(shooter);

        // load field artifacts
        Actions.runBlocking(slurpArtifacts(drive, 0.5 * gridSize));
        shoot(shooter);
        Actions.runBlocking(slurpArtifacts(drive, -0.5 * gridSize));
        shoot(shooter);
        Actions.runBlocking(slurpArtifacts(drive, -1.5 * gridSize));
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

    private Action slurpArtifacts(MecanumDrive drive, double posY) {
        return drive.actionBuilder(startingPos)
                .lineToX(.5 * gridSize)
                .turn(Math.toRadians(35))
                .lineToY(posY)
                .turn(Math.toRadians(80))
                .lineToX(2.3 * gridSize)
                .setReversed(true)
                .lineToX(0.5 * gridSize)
                .turn(Math.toRadians(-90))
                .lineToY(0.5 * gridSize)
                .turn(Math.toRadians(-35))
                .lineToX(startingPos.position.x)
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


package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorProcessor.DetectedColor;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.example.SmartShooter;


@Disabled
@Autonomous
public class BlueTokyoDrift2 extends LinearOpMode {
    // grid size is handy for describing distances
    final double gridSize = 23.625;
    // starting position - backed up to goal, ready to shoot
    final Pose2d startingPos = new Pose2d(-2 * gridSize, 2.2 * gridSize, Math.toRadians(-37));

    MecanumDrive drive;
    Limelight limelight;

    SmartShooter shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, startingPos);
        drive.localizer.setPose(startingPos);
        shooter = new SmartShooter(hardwareMap);
        limelight = new Limelight(hardwareMap);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        // Wait for the driver to press start
        waitForStart();

        // start intake and flywheel
        shooter.setMotorVelocity(330);
        Thread.sleep(1500);
        intake.setVelocity(-1575);
        intake2.setVelocity(-1575);
        Thread.sleep(1000);

        // shoot preloaded artifacts
        shoot(shooter);

        // load field artifacts
        runAction(slurpArtifacts(drive, 0.5 * gridSize));
        shoot(shooter);
        runAction(slurpArtifacts(drive, -0.4 * gridSize));
        shoot(shooter);

        // leave line
        Actions.runBlocking(
                drive.actionBuilder(startingPos)
                        .setTangent(Math.toRadians(-90))
                        .lineToY(0)
                        .build()
        );

        // turn off intake and flywheel
        shooter.setPower(0);
        intake.setPower(0);
    }

    private void runAction(Action trajectoryAction) {
        boolean running = true;
        while (opModeIsActive() && running) {
            drive.updatePoseEstimate();
            limelight.updateTags();
            telemetry.addData("tags: ", limelight.observedAprilTags.toString());
            telemetry.update();
            running = trajectoryAction.run(new TelemetryPacket());
        }
    }

    private Action slurpArtifacts(MecanumDrive drive, double posY) {
        double wallX = (-2.0 * gridSize);
        if (posY < 0.2 * gridSize){
            wallX = (-2.7 * gridSize);
        }
        return drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(
                        new Pose2d(-1 * gridSize, posY + 3, Math.toRadians(180)),
                        Math.toRadians(-90)
                )
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(wallX, posY - 2), Math.toRadians(195))
                .setTangent(Math.toRadians(0))
                .lineToX((startingPos.position.x) + 6)
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(startingPos.position.y, startingPos.heading)
                .setTangent(Math.toRadians(180))
                .lineToX(startingPos.position.x)
                .build();
    }

    private void shoot(SmartShooter shooter) {
        // functions we can use:
        // limelight.hasSeenTag(21)
        // shooter.shootColor(DetectedColor.PURPLE);
    }
}


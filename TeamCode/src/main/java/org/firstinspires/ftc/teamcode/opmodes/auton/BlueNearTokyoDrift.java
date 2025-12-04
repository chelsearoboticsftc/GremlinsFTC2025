package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.example.SmartShooter;


@Autonomous
public class BlueNearTokyoDrift extends LinearOpMode {
    // grid size is handy for describing distances
    final double gridSize = 23.625;
    // starting position - backed up to goal, ready to shoot
    final Pose2d startingPos = new Pose2d(-2 * gridSize, 2.2 * gridSize, Math.toRadians(-37));

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPos);
        drive.localizer.setPose(startingPos);
        SmartShooter shooter = new SmartShooter(hardwareMap);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        limelight = new Limelight(hardwareMap);


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
        runAction(drive, slurpArtifacts(drive, -0.3 * gridSize));
        shoot(shooter);
        Actions.runBlocking(slurpArtifacts(drive, -0.3 * gridSize));
        shoot(shooter);
        Actions.runBlocking(slurpArtifacts(drive, -1.2 * gridSize));
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

    private void runAction(MecanumDrive drive, Limelight limelight, Action trajectoryAction) {
        boolean running = true;
        while (opModeIsActive() && running) {
            drive.updatePoseEstimate();
            limelight.updatePose(drive);
            running = trajectoryAction.run(new TelemtryPacket());
        }
    }

    private Action slurpArtifacts(MecanumDrive drive, double posY) {
        double wallX = (-2.05 * gridSize);
        if (posY < 0.2 * gridSize){
            wallX = (-2.25 * gridSize);
        }
        return drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(
                        new Pose2d(-1 * gridSize, posY, Math.toRadians(180)),
                        Math.toRadians(-90)
                )
                .setTangent(Math.toRadians(180))
                .lineToX(wallX)
                .setTangent(Math.toRadians(0))
                .lineToX((startingPos.position.x) + 3)
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(startingPos.position.y, startingPos.heading)
                .setTangent(Math.toRadians(180))
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


package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorProcessor;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorProcessor.DetectedColor;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.example.SmartShooter;


@TeleOp
public class TestForKirk4 extends LinearOpMode {
    public enum ShootingState {
        IDLE,
        RAISING_GATE,
        LOWERING_GATE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        final double gridSize = 23.5;
        final Pose2d startingPos = new Pose2d(-2 * gridSize, 2.2 * gridSize, Math.toRadians(-37));
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        drive.localizer.setPose(startingPos);
        DcMotorEx intake1 = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        SmartShooter shooter = new SmartShooter(hardwareMap);
        ColorSensorProcessor colorSensorLeft = new ColorSensorProcessor(hardwareMap, "colorSensorLeft");
        ColorSensorProcessor colorSensorRight = new ColorSensorProcessor(hardwareMap, "colorSensorRight");
        Limelight limelight = new Limelight(hardwareMap);

        ElapsedTime shootingTimer = new ElapsedTime();
        ShootingState currentShootingState = ShootingState.IDLE;
        boolean shootLeftGate = false; // Flag to remember the order
        boolean shootRightGate = false;
        boolean intakeon = false;

        waitForStart();

        while(opModeIsActive()) {
            drive.localizer.update();

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
                // toggle intake power
                intakeon = !intakeon;

            if (this.gamepad2.x) {
                // run intake in reverse
                intake1.setVelocity(1575);
                intake2.setVelocity(1575);
            }
            else {
                if (intakeon) {
                    intake1.setVelocity(-1575);
                    intake2.setVelocity(-1575);
                    // start flywheel when the intake is running
                    shooter.setMotorVelocity(340);
                }
                else {
                    intake1.setPower(0);
                    intake2.setPower(0);
                    shooter.setPower(0);
                }
            }

            switch (currentShootingState) {
                case IDLE:
                    if (gamepad2.leftBumperWasPressed()) {
                        shootLeftGate = (colorSensorLeft.getDetectedColor() == DetectedColor.PURPLE);
                        shootRightGate = (colorSensorRight.getDetectedColor() == DetectedColor.PURPLE);
                        currentShootingState = ShootingState.RAISING_GATE;
                    } else if (gamepad2.rightBumperWasPressed()) {
                        // Shoot green first
                        shootLeftGate = (colorSensorLeft.getDetectedColor() == DetectedColor.GREEN);
                        shootRightGate = (colorSensorRight.getDetectedColor() == DetectedColor.GREEN);
                        currentShootingState = ShootingState.RAISING_GATE;
                    } else {
                        shooter.lowerGates();
                    }
                    break;

                case RAISING_GATE:
                    if (shootLeftGate) {
                        shooter.raiseLeftGate();
                    } else if (shootRightGate) {
                        shooter.raiseRightGate();
                    } else {
                        if (colorSensorRight.getDetectedColor() != DetectedColor.NONE) {
                            shooter.raiseRightGate();
                        } else {
                            shooter.raiseLeftGate();
                        }
                    }
                    shootingTimer.reset(); // Start the timer for the delay
                    currentShootingState = ShootingState.LOWERING_GATE;
                    break;

                case LOWERING_GATE:
                    // Wait for another 500ms before lowering both gates and returning to idle.
                    if (shootingTimer.milliseconds() > 400) {
                        shooter.lowerGates();
                        currentShootingState = ShootingState.IDLE; // Sequence is complete, go back to IDLE
                    }
                    break;
            }

            telemetry.addData("Shooting State", currentShootingState);
            telemetry.addData("Detected colorL", colorSensorLeft.getDetectedColor());
            telemetry.addData("Detected colorR", colorSensorRight.getDetectedColor());
            telemetry.addData(
                    "RGB",
                    String.format("%d/%d/%d",
                        colorSensorLeft.colorSensor.red(),
                        colorSensorLeft.colorSensor.green(),
                        colorSensorLeft.colorSensor.blue()
                    )
            );
            telemetry.addData(
                    "HSV",
                    String.format("%.1f/%.2f/%.2f",
                            colorSensorLeft.hsvValues[0],
                            colorSensorLeft.hsvValues[1],
                            colorSensorLeft.hsvValues[2]
                    )
            );
            for (Limelight.TagDetection detection : limelight.getCurrentDetections()) {
                telemetry.addData("Detection", detection.toString());
            }

            telemetry.update();
        }
    }
}

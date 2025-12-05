package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.example.SmartShooter;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorProcessor;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorProcessor.DetectedColor;

@Disabled
@TeleOp
public class TestForKirk3 extends LinearOpMode {
    public enum ShootingState {
        IDLE,
        RAISING_FIRST_GATE,
        RAISING_SECOND_GATE,
        LOWERING_GATES
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        DcMotorEx intake1 = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        SmartShooter shooter = new SmartShooter(hardwareMap);
        ColorSensorProcessor colorSensorLeft = new ColorSensorProcessor(hardwareMap, "colorSensorLeft");
//        Limelight limelight = new Limelight(hardwareMap);

        ElapsedTime shootingTimer = new ElapsedTime();
        ShootingState currentShootingState = ShootingState.IDLE;
        boolean shootLeftGateFirst = false; // Flag to remember the order
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
                    shooter.setMotorVelocity(350);
                }
                else {
                    intake1.setPower(0);
                    intake2.setPower(0);
                    shooter.setPower(0);
                }
            }

            switch (currentShootingState) {
                case IDLE:
                    // In the IDLE state, we wait for a button press to start the sequence.
                    // We only listen for buttons if we are not already in a shooting sequence.
                    if (gamepad2.leftBumperWasPressed()) {
                        // Operator requested to shoot purple first
                        // Determine which side to shoot first based on the color sensor
                        // Note - we don't even need to check the 2nd color sensor, because if we don't
                        // have the requested color in the side we check, we might as well just
                        // start with the other side - checking if it has the requested color
                        // doesn't change anything.
                        shootLeftGateFirst = (colorSensorLeft.getDetectedColor() == DetectedColor.PURPLE);
                        currentShootingState = ShootingState.RAISING_FIRST_GATE;
                    } else if (gamepad2.rightBumperWasPressed()) {
                        // Shoot green first
                        shootLeftGateFirst = (colorSensorLeft.getDetectedColor() == DetectedColor.GREEN);
                        currentShootingState = ShootingState.RAISING_FIRST_GATE;
                    }
                    break;

                case RAISING_FIRST_GATE:
                    // This state is entered immediately from IDLE.
                    // Raise the correct first gate, reset the timer, and move to the next state.
                    if (shootLeftGateFirst) {
                        shooter.raiseLeftGate();
                    } else {
                        shooter.raiseRightGate();
                    }
                    shootingTimer.reset(); // Start the timer for the delay
                    currentShootingState = ShootingState.RAISING_SECOND_GATE;
                    break;

                case RAISING_SECOND_GATE:
                    // Wait for 2000ms (2 seconds) before raising the second gate.
                    if (shootingTimer.milliseconds() > 2000) {
                        // Time's up, raise the other gate
                        if (shootLeftGateFirst) {
                            shooter.raiseRightGate();
                        } else {
                            shooter.raiseLeftGate();
                        }
                        shootingTimer.reset(); // Reset timer for the final delay
                        currentShootingState = ShootingState.LOWERING_GATES;
                    }
                    break;

                case LOWERING_GATES:
                    // Wait for another 2000ms before lowering both gates and returning to idle.
                    if (shootingTimer.milliseconds() > 2000) {
                        shooter.lowerGates();
                        currentShootingState = ShootingState.IDLE; // Sequence is complete, go back to IDLE
                    }
                    break;
            }

            telemetry.addData("Shooting State", currentShootingState);
            telemetry.addData("Detected color", colorSensorLeft.getDetectedColor());
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
//            for (Limelight.TagDetection detection : limelight.getCurrentDetections()) {
//                telemetry.addData("Detection", detection.toString());
//            }
            telemetry.update();
        }
    }
}

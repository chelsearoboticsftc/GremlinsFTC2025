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
public class ColorTest extends LinearOpMode {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensorProcessor colorSensorLeft = new ColorSensorProcessor(hardwareMap, "colorSensorLeft");
        waitForStart();

        while(opModeIsActive()) {

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
            telemetry.update();
        }
    }
}

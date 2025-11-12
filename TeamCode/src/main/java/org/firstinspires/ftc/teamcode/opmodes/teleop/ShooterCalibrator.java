package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.example.SmartShooter;
import org.firstinspires.ftc.teamcode.subsystems.example.SmartShooterConstants;

@TeleOp
public class ShooterCalibrator extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry tel = dashboard.getTelemetry();
        SmartShooter shooter = new SmartShooter(hardwareMap);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterLeft");

        waitForStart();
//        intake.setPower(-1);
        double step = 20;
        double velocity = 0;

        while(opModeIsActive()){
            // A button presses increase velocity by "step"
            if (gamepad2.aWasPressed()) {
                velocity += step;
            }
            // B button presses reset velocity to 0
            if (gamepad2.bWasPressed()) {
                velocity = 0;
            }
            shooter.setMotorVelocity(velocity);
            tel.addData("Commanded", velocity);
            tel.addData("Actual", shooterMotor.getVelocity(AngleUnit.DEGREES));
            tel.update();

            if (gamepad2.left_bumper)
                shooter.raiseLeftGate();

            if (gamepad2.left_trigger > 0.1)
                shooter.lowerLeftGate();

            if(gamepad2.right_bumper)
                shooter.raiseRightGate();

            if (gamepad2.right_trigger > 0.1)
                shooter.lowerRightGate();
        }
    }
}

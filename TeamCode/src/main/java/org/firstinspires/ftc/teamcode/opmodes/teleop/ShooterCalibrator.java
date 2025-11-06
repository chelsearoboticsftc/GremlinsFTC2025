package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.example.SmartShooter;

@TeleOp
public class ShooterCalibrator extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SmartShooter shooter = new SmartShooter(hardwareMap);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

        waitForStart();
        intake.setPower(-0.7);
        shooter.raiseGates();
        double step = 20;
        double velocity = 500;

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
            this.telemetry.addData("Velocity", velocity);
            this.telemetry.update();
        }
    }
}

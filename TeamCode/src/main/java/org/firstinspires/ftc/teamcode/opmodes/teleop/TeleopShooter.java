package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.example.SampleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.example.SampleSubsystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.example.shooter;
@TeleOp
public abstract class TeleopShooter extends LinearOpMode {

    public void runOpMode() throws InterruptedException{

        shooter subsystem = new shooter(hardwareMap);

        //Call init() method to do any initialization actions defined by the subsystem class
        subsystem.init();

        waitForStart();

        while(opModeIsActive()){
            //Go to A Position if A pressed, go to B Position if B Pressed
            if(gamepad1.yWasPressed()){
                subsystem.setMotorPosition(SampleSubsystemConstants.MOTOR_NAME_A_POSITION);
            } else if (gamepad1.aWasPressed()){
                subsystem.setMotorPosition(SampleSubsystemConstants.MOTOR_NAME_B_POSITION);
            } else if (gamepad1.x) {
                subsystem.setMotorPower(0.5);
            } else if (gamepad1.b) {
                subsystem.setMotorPower(-0.5);
            } else if ((!gamepad1.x)&&(!gamepad1.b)){
                subsystem.setMotorPower(0);
            } else {
    }
}
    }}

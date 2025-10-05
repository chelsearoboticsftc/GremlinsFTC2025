package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.example.SampleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.example.SampleSubsystemConstants;

@Disabled
public class SampleTestSubsystemTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleSubsystem subsystem = new SampleSubsystem(hardwareMap);

        //Call init() method to do any initialization actions defined by the subsystem class
        subsystem.init();

        //DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motorName");

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
                //Not needed but shows we do nothing if neither button was pressed!
            }

            //Call subsystem update method
            subsystem.update();

            //Update Telemetry
            telemetry.addData("A Pressed", gamepad1.x);
            telemetry.addData("B Pressed", gamepad1.b);
            telemetry.addData("Motor Actual Position", subsystem.getMotorPosition());
            telemetry.addData("Motor Set Position", subsystem.getMotorTargetPosition());
            telemetry.addData("Motor Power", subsystem.getMotorPower());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.example.SampleVision;

@TeleOp
public class SampleVisionTester extends LinearOpMode {

    private double distance;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleVision GoalDistance = new SampleVision(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            distance = GoalDistance.getDistanceToGoal(20);
            //20 equals BLUE team
            telemetry.addLine(String.format("range %6.1f inch",
                    distance));
            telemetry.update();
            sleep(5000);
        }
    }

}

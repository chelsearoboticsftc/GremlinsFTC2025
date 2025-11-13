package org.firstinspires.ftc.teamcode.subsystems.example;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.LookupTable;

public class SmartShooter {

    //Example declare a DcMotorEx object as part of this class called 'motorName'
    DcMotorEx motor;
    Servo upperLeftGate;
    Servo upperRightGate;

    //Declare any other global variables for this class here
    private final LookupTable distanceToVelocity = new LookupTable(SmartShooterConstants.LOOKUP_TABLE);

    public SmartShooter(HardwareMap hardwareMap){
        this.motor = hardwareMap.get(DcMotorEx.class, SmartShooterConstants.MOTOR_NAME);
        this.motor.setDirection(SmartShooterConstants.MOTOR_DIRECTION);
        this.upperLeftGate = hardwareMap.get(Servo.class, "upperLeftGate");
        this.upperRightGate = hardwareMap.get(Servo.class, "upperRightGate");
        this.lowerGates();

        //This defines the behavior at zero power (brake or coast)
        motor.setZeroPowerBehavior(SmartShooterConstants.ZERO_POWER_BEHAVIOR);

        //This defines the motor direction (forward or reversed)
        motor.setDirection(SmartShooterConstants.MOTOR_DIRECTION);

        /* This defines the motor velocity PIDF gains.  Velocity PIDF values determine control    *
         * around a target velocity (setTargetVelocity) OR how fast the system responds to a      *
         * change in set position (setTargetPosition).                                            */
        motor.setVelocityPIDFCoefficients(
                SmartShooterConstants.VELOCITY_P, //Proportional Gain
                SmartShooterConstants.VELOCITY_I, //Integral Gain
                SmartShooterConstants.VELOCITY_D, //Derivative Gain
                SmartShooterConstants.VELOCITY_F);//Feed Forward Gain

        /* This defines the motor position PID P gain. Position control only needs P gain since   *
         * once the system reaches the target position since once at position you're only         *
         * disturbances in the system                                                             */
        motor.setPositionPIDFCoefficients(
                SmartShooterConstants.POSITION_P);//Proportional Gain

        //motorName.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    /* Standard functions.  All Chelsea Robotics subsystems shall have init() and update() these  *
     * methods defined. Leave empty if not needed!                                                */
    public void init(){
        /* Call this method at the start of your opmode logic once to execute any logic you       *
         * want to be called on initialization. If none, leave empty!                             */
    }

    public void update(){
        //Call this method each time your opmode logic loops (i.e. inside while(opModeIsActive()){}
        //to execute any logic you want to be called periodically. If none, leave empty!

        //setTargetPosition needs to be called once per loop to keep the REV watchdog happy
        //motorName.setTargetPosition(motorSetPosition);
    }

    public void raiseLeftGate() {
        upperLeftGate.setPosition(0.5);
    }
    public void raiseRightGate() {
        upperRightGate.setPosition(0.4);
    }
    public void raiseGates(){
        this.raiseLeftGate();
        this.raiseRightGate();
    }

    public void lowerLeftGate() {
        upperLeftGate.setPosition(1);
    }

    public void lowerRightGate() {
        upperRightGate.setPosition(0.0);
    }
    public void lowerGates(){
        this.lowerLeftGate();
        this.lowerRightGate();
    }

    public void shoot(double velocity, boolean left, boolean right) {
        if (left && right) {
            return;
        }
//        double velocity = distanceToVelocity.interpolate(distance);
//        this.setMotorVelocity(velocity);
        // TODO - is ball already engaged, or does it need to be dropped,
        // maybe after a short delay to allow the motor to spin up?
        this.setMotorVelocity(320);
        try {
            Thread.sleep(4000);
        } catch (InterruptedException e) {
        }

        if (left) {
            this.raiseLeftGate();
        }
        if (right) {
            this.raiseRightGate();
        }

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
        }
        this.motor.setVelocity(0);
        this.lowerGates();
    }

    public void setPower(double power) {
        this.motor.setPower(power);
    }

    public void setMotorVelocity(double angularRate) {
        this.motor.setVelocity(angularRate, AngleUnit.DEGREES);
    }
}
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Meccanum_drive {

    private DcMotor leftFront, leftBack, rightFront, rightBack;


    public void init(HardwareMap hwmap) {
        leftFront = hwmap.get(DcMotor.class, "leftFront");
        leftBack = hwmap.get(DcMotor.class, "leftBack");
        rightFront = hwmap.get(DcMotor.class, "rightFront");
        rightBack = hwmap.get(DcMotor.class, "rightBack");

        //Set motor Directions - 10/15/26
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    //declare drive variables to be called in other classes - 10/15/26
    public void drive(double forward, double strafe, double rotate) {
        double frontleftpower = forward + strafe + rotate;
        double backleftpower = forward - strafe + rotate;
        double frontrightpower = forward - strafe - rotate;
        double backrightpower = forward + strafe - rotate;

        double maxPower = 1.0; //Sets max motor power
        double maxSpeed = 1.0; //Adjusts robot speed - set lower for outreach

        //set maxPower to motors - 10/15/26
        maxPower = Math.max(maxPower, Math.abs(frontleftpower));
        maxPower = Math.max(maxPower, Math.abs(backleftpower));
        maxPower = Math.max(maxPower, Math.abs(frontrightpower));
        maxPower = Math.max(maxPower, Math.abs(backrightpower));

        //Constrain with max speed - 10/15/26
        leftFront.setPower(maxSpeed * (frontleftpower / maxPower));
        leftBack.setPower(maxSpeed * (backleftpower / maxPower));
        rightFront.setPower(maxSpeed * (frontrightpower / maxPower));
        rightBack.setPower(maxSpeed * (backrightpower / maxPower));
    }
}

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

        leftFront.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setDirection(DcMotor.Direction.FORWARD);

        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void drive(double forward, double strafe, double rotate) {
        double frontleftpower = forward + strafe + rotate;
        double backleftpower = forward - strafe + rotate;
        double frontrightpower = forward - strafe - rotate;
        double backrightpower = forward + strafe - rotate;

        double maxPower = 1.0; //Sets max motor power
        double maxSpeed = 1.0; //Adjusts robot speed

        maxPower = Math.max(maxPower, Math.abs(frontleftpower));
        maxPower = Math.max(maxPower, Math.abs(backleftpower));
        maxPower = Math.max(maxPower, Math.abs(frontrightpower));
        maxPower = Math.max(maxPower, Math.abs(backrightpower));

        leftFront.setPower(maxSpeed * (frontleftpower / maxPower));
        leftBack.setPower(maxSpeed * (backleftpower / maxPower));
        rightFront.setPower(maxSpeed * (frontrightpower / maxPower));
        rightBack.setPower(maxSpeed * (backrightpower / maxPower));
    }
}

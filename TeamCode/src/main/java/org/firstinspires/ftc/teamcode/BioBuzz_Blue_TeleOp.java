package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mechanisms.Meccanum_drive;

@TeleOp (name = "BioBuzz_Blue_TeleOp", group = "BioBuzz_TeleOp")
public class BioBuzz_Blue_TeleOp extends OpMode {

    private Limelight3A limelight;

    GoBildaPinpointDriver imu;

    //---Meccanum---
    final Meccanum_drive drive = new Meccanum_drive();

    double forward, strafe, rotate;

    //---Launcher & Intake---

    //ms(main shooter) lw(load wheel) mi(main intake)
    private DcMotor ms, lw, mi;

    //---LED---
    private Servo led_chain;

    //---PD Controller---

    double kP = 0.0500;
    double error = 0;
    double lastError =0;
    double goalX = 0;
    double angleTolerance = 0.4;
    double kD = 0.0030;
    double curTime = 0;
    double lastTime = 0;

    //---Rumble---
    boolean secondHalf = false;

    boolean endgame = false;

    ElapsedTime time = new ElapsedTime();

    final double HALF_TIME = 60.0;
    final double END_GAME = 90.0;

    @Override
    public void init() {
        //---Launcher---
        ms = hardwareMap.get(DcMotor.class, "ms");
        lw = hardwareMap.get(DcMotor.class, "lw");
        mi = hardwareMap.get(DcMotor.class, "mi");

        ms.setDirection(DcMotor.Direction.FORWARD);
        lw.setDirection(DcMotor.Direction.FORWARD);
        mi.setDirection(DcMotor.Direction.FORWARD);

        //Encoder control means the motor will target a speed and will be less affected by battery power
        ms.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //---Meccanum---
        drive.init(hardwareMap);

        //---LimeLight---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        telemetry.setMsTransmissionInterval(11);

        //---PinPoint---
        imu = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        imu.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));

        //---LED---
        led_chain = hardwareMap.get(Servo.class, "led_chain");
    }

    public double getHeading() {
        return imu.getHeading(AngleUnit.RADIANS);
    }

    @Override
    public void start() {
        limelight.start();
        time.reset();    // Start game timer.
    }

    @Override
    public void loop() {
        //---GamePad 1---
        //---Mechanum Drive Inputs---
        forward = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        limelight.updateRobotOrientation(getHeading());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("yaw", botPose.getOrientation());
            telemetry.addData("botpose", botPose.toString());
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        //---Get April tag info---
        assert llResult != null;
        double tx = llResult.getTx();
        //---Auto Align---
        if (gamepad1.left_trigger > 0.3){
            if (tx != 0) {
                error = goalX - tx;

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = error * kP;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error - lastError) / dT) * kD;

                    rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                lastError = 0;
                lastTime = getRuntime();
            }
            //rumble operator controller when tracking - 10/22/26
            gamepad2.rumble(150);
        } else {
            lastError = 0;
            lastTime = getRuntime();
        }

        {
            drive.drive(forward, strafe, rotate);
        }

        //---GamePad 2---
        //---Main Shooter---
        if (gamepad2.left_trigger > 0.3){
            ms.setPower(0.54); //5375 okay - 0.60 too powerful
        } else {
            ms.setPower(0);
        }
        //---Load Wheels---
        if (gamepad2.right_trigger > 0.3){
            lw.setPower(1);
        }else{
            lw.setPower(0);
        }
        //---Unload Wheels---
        if (gamepad2.left_bumper){
            lw.setDirection(DcMotor.Direction.REVERSE);
        }else{
            lw.setDirection(DcMotor.Direction.FORWARD);
        }
        //---Main Intake---
        if (gamepad2.right_bumper){
            mi.setPower(1);
            gamepad1.rumble(150);
        }else{
            mi.setPower(0);
        }
        //---Out-take---
        if (gamepad2.left_bumper){
            mi.setDirection(DcMotor.Direction.REVERSE);
        }else{
            mi.setDirection(DcMotor.Direction.FORWARD);
        }

        //---Rumble---
        if ((time.seconds() > HALF_TIME) && !secondHalf)  {
            gamepad1.rumble(500);
            gamepad2.rumble(500);
            secondHalf =true;
        }

        if ((time.seconds() > END_GAME) && !endgame) {
            gamepad1.rumble(500);
            gamepad2.rumble(500);
            endgame =true;
        }

        //---LED---
        led_chain.setPosition(0.611); //PWM LED's take a servo position command to set color

        //---Telemetry---
        telemetry.addLine("---Telemetry---");
        telemetry.addData("Time elapsed", "%3.0f", time.seconds());
        telemetry.addData("Shooter Power", ms.getPower());
        telemetry.addData("Intake Power", mi.getPower());
        telemetry.addData("Load Wheel Power", lw.getPower());
        telemetry.addData("ms position", ms.getCurrentPosition());
    }

    public void configurePinpoint(){
        //Odometry pod positions
        imu.setOffsets(-22.0, -155.0, DistanceUnit.MM);
        //Pod Types
        imu.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //Pod directions
        imu.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //IMU auto calibrate
        imu.resetPosAndIMU();
    }


}

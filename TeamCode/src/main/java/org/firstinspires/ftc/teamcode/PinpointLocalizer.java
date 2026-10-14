package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class PinpointLocalizer implements Localizer {
    // Road Runner's TuningOpMode explicitly looks for a public field named "driver"
    public final GoBildaPinpointDriver driver;
    private Pose2d pose;

    // FIX: Set these constants to use the specific EncoderDirection object type instead of integers
    public static GoBildaPinpointDriver.EncoderDirection initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set your encoder resolution (e.g., goBILDA 4-bar pod defaults)
        driver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Pass our static configuration directions straight into the driver initialization
        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        // Pass DistanceUnit.MM explicitly
        driver.setOffsets(0.0, 0.0, DistanceUnit.MM);

        driver.resetPosAndIMU();

        // Set the initial starting position
        setPose(initialPose);
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
        driver.setPosition(new Pose2D(DistanceUnit.INCH, pose.position.x, pose.position.y, AngleUnit.RADIANS, pose.heading.toDouble()));
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();

        // 1. Get current position directly in Inches and Radians
        Pose2D pinpointPose = driver.getPosition();
        pose = new Pose2d(
                pinpointPose.getX(DistanceUnit.INCH),
                pinpointPose.getY(DistanceUnit.INCH),
                pinpointPose.getHeading(AngleUnit.RADIANS)
        );

        // 2. Get velocities
        double velXInches = driver.getVelX(DistanceUnit.INCH);
        double velYInches = driver.getVelY(DistanceUnit.INCH);
        double headingVel = driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        return new PoseVelocity2d(
                new Vector2d(velXInches, velYInches),
                headingVel
        );
    }
}

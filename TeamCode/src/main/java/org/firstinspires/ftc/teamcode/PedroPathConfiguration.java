package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamPedroPathing.SwyftMecanum;

/**
 * This class provides a single location to set Pedro Path's myriad constraints.
 * It is responsible for creating and returning a Follower built using these constraints.
 */
public class PedroPathConfiguration {
    private final OpMode myOpMode;

    private Follower follower;

    public PedroPathConfiguration(LinearOpMode opMode) {
        this.myOpMode = opMode;
        init();
    }

    /**
     * Call all the constant builders and build the Follower instance.
     */
    private void init() {
        MecanumConstants driveConstants = buildMecanumConstants();

        this.follower = new FollowerBuilder(buildFollowerConstants(), myOpMode.hardwareMap)
                .setDrivetrain(new SwyftMecanum(myOpMode.hardwareMap, driveConstants))
                .pinpointLocalizer(buildPinpointConstants())
                .pathConstraints(buildPathConstraints())
                .build();

        /*
         * Follower has its own globalMaxPower that is initialized to 1.
         * Irritatingly, this cannot be changed using FollowerConstants.
         * It can only be changed by calling setMaxPower() on the built instance
         * of Follower. Set globalMaxPower to be the the same as that in DriveConstants.
         * (DriveConstants are set via MecanumConstants.)
         */
        this.follower.setMaxPower(driveConstants.getMaxPower());
    }

    private FollowerConstants buildFollowerConstants() {
        return new FollowerConstants()
                .mass(8.8)
                .forwardZeroPowerAcceleration(-26.27)
                .lateralZeroPowerAcceleration(-66.36)
                .headingPIDFCoefficients(new PIDFCoefficients(.7, 0, 0, .02))
                .translationalPIDFCoefficients(new PIDFCoefficients(.05, 0, 0, .02))
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(.01, 0, .00001, .6, .001 ))
                .centripetalScaling(0.0006);
    }

    private MecanumConstants buildMecanumConstants() {
        return new MecanumConstants()
                .maxPower(1)
                .rightFrontMotorName("RightFront")
                .rightRearMotorName("RightRear")
                .leftRearMotorName("LeftRear")
                .leftFrontMotorName("LeftFront")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .xVelocity(73.15)
                .yVelocity(54.63);
    }

    private PinpointConstants buildPinpointConstants() {
        return new PinpointConstants()
                .forwardPodY(-.875)
                .strafePodX(-.75)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("pinPoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    private PathConstraints buildPathConstraints() {
        return new PathConstraints(.99, 100, .75, 1);
    }

    public Follower getFollower() {
        return follower;
    }
}

package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroTeleopData;

/**
 * This OpMode demonstrates using Pedro Pathing for teleOp,
 * and also building and a Path to "instantly" go to the launch Pose
 * from anywhere on the field.
 */
@TeleOp(name = "Pedro Path Teleop")
public class PedroPathTeleOp extends LinearOpMode {
    /*
     * The preceding Autonomous OpMode should set this static startPose
     * with the Pose it stopped at (follower.getPose()).
     */
    private Pose parkPose;
    private Follower follower;

    private Pose launchPose;
    private Pose defaultStartingPose;
    private int shooterState = 0;
    private String shooterDistance = "None";
    private RobotHardware robot;
    private AllianceColor allianceColor;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(this);

        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);

        allianceColor = PedroTeleopData.allianceColor == null ? AllianceColor.BLUE :
            PedroTeleopData.allianceColor;
        initSetup();

        follower = pedroPathConfiguration.getFollower();
        setPoses(allianceColor);

        Pose startPose = PedroTeleopData.startingPose == null ? defaultStartingPose :
                PedroTeleopData.startingPose;
        follower.setStartingPose(startPose);
        telemetry.addLine("Starting Pose:");
        telemetry.addData("x", startPose.getX());
        telemetry.addData("y", startPose.getY());
        telemetry.addData("heading", Math.toDegrees(startPose.getHeading()));
        telemetry.update();

        follower.startTeleOpDrive(); //This calls update() as well.

        waitForStart();

        if (opModeIsActive()) {
            robot.startSpinTake();
            robot.startSideMotor();
        }

        /*
         * The follower can be either in startTeleOpDrive() or followPath().
         */
        while (opModeIsActive()) {
            follower.update();

            //Call setTeleOpDrive() as long as isTeleopDrive().
            if (follower.isTeleopDrive()) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            }
            /*
             * If not isTeleopDrive(), then we might still be following a Path.
             * If that Path is complete (not isBusy()), do whatever we went there to do,
             * then re-startTeleOpDrive().
             */
            else if (!follower.isBusy()) {
                //shootBalls();
                follower.startTeleOpDrive();
            }

            /*
             * See if we pressed the gamepad button to follow a Path.
             * If whatever Path we might already have been on is complete (not isBusy()),
             * followPath() the path fromHereToLaunch().
             */
            if (gamepad1.aWasPressed()) {
                if (follower.isBusy()) {
                    follower.startTeleOpDrive();
                }
                else {
                   follower.followPath(fromHereToLaunch());
                }
            }

            if (gamepad1.x) {
                toggleShooterDistance();
                sleep(500); // debounce
            }

            if (gamepad1.yWasPressed()) {
                robot.setTopConveyPower(1);
            } else if (gamepad1.yWasReleased()){
                robot.setTopConveyPower(0);
            }
        }
    }

    /**
     * Build a PathChain from our current Pose to the launchPose,
     * and set it to LinearHeadingInterpolation.
     *
     * @return PathChain
     */
    private PathChain fromHereToLaunch() {
        Pose herePose = follower.getPose();

        return follower.pathBuilder()
                .addPath(new BezierLine(herePose, launchPose))
                .setLinearHeadingInterpolation(herePose.getHeading(), launchPose.getHeading())
                .build();
    }

    private void toggleShooterDistance() {
        switch (shooterState) {
            case 0:
                robot.setShooterPower(0.40);
                shooterDistance = "Near";
                shooterState = 1;
                break;

            case 1:
                robot.setShooterPower(0.50);
                shooterDistance = "Far";
                shooterState = 0;
                break;
        }

        telemetry.addData("Shooter Distance", shooterDistance);
    }

    private void initSetup() {
        while (opModeInInit()) {
            telemetry.addLine("Press Right Bumper to toggle between Red and Blue alliance.");
            telemetry.addData(allianceColor.toString(), " currently selected");
            telemetry.update();
            allianceColor = AllianceColor.INSTANCE.toggleColor(gamepad1.rightBumperWasPressed(), allianceColor);
        }
    }

    private void setPoses(AllianceColor allianceColor) {
        GirlsPedroPathsFrontWall paths = (allianceColor == AllianceColor.RED) ?
                new GirlsRedPedroPathsFrontWall(follower, new GirlsBluePedroPathsFrontWall(follower)) :
                new GirlsBluePedroPathsFrontWall(follower);

        launchPose = paths.pathFromEndAudiencePickupToLaunchZone().endPose();
        parkPose = paths.parkPose();
        defaultStartingPose = paths.farStartingPose();
    }
}
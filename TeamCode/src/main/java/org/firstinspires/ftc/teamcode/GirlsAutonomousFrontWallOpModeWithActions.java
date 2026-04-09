package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathTelemetry;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroTeleopData;

@Autonomous(name = "Pedro Autonomous")
public class GirlsAutonomousFrontWallOpModeWithActions extends LinearOpMode {
    private PedroPathTelemetry pedroTelemetry;
    private Follower follower;
    private final double ballPickupPower = .4;
    private AllianceColor selectedColor = AllianceColor.RED;
    private FrontBackLocation selectedStartPosition = FrontBackLocation.FRONT;
    private FrontBackLocation selectedLaunchPosition = FrontBackLocation.FRONT;
    private int pathState = 0;
    private String pedroMessage = "NO MESSAGE";
    private GirlsPedroPathsFrontWall pedroPaths;
    private Pose startPose;
    private PathChain pathFromStartToLaunchZone;
    private PathChain pathFromLaunchZoneToStartBallPickup1;
    private PathChain pathFromStartBallPickupToEndBallPickup1;
    private PathChain pathFromEndBallPickupToLaunchZone1;
    private PathChain pathFromLaunchZoneToStartBallPickup2;
    private PathChain pathFromStartBallPickupToEndBallPickup2;
    private PathChain pathFromEndBallPickupToLaunchZone2;

    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(this);

        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);

        follower = pedroPathConfiguration.getFollower();
        initSetup();

        PedroTeleopData.allianceColor = selectedColor;
        pedroPaths = createPedroPaths(selectedColor);
        setBallSpikeLocationPaths();
        setStartPoseAndPath();
        pedroTelemetry = new PedroPathTelemetry(telemetry, follower, selectedColor);
        follower.setStartingPose(startPose);

        waitForStart();

        boolean done = false;
        while (opModeIsActive() && !done) {
            follower.update();
            done = performActions();
            pedroTelemetry.pathTelemetry(pedroMessage);
        }

        PedroTeleopData.startingPose = follower.getPose();
    }

    private boolean performActions() {
        boolean done = false;
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    shooterVelocity(2800 * .3);
                    pedroMessage = "Going from wall to launch zone";
                    follower.followPath(pathFromStartToLaunchZone );
                    pathState++;
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    robot.startSpinTake();
                    shootBalls();
                    pedroMessage = "Going from launch zone to ball pickup 1";
                    follower.followPath(pathFromLaunchZoneToStartBallPickup1);
                    pathState++;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    pedroMessage = "Ingesting balls 1";
                    follower.followPath(pathFromStartBallPickupToEndBallPickup1, ballPickupPower, true);
                    pathState++;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    shooterVelocity(2800 * .3);
                    pedroMessage = "Going from ball pickup to launch zone 1";
                    follower.followPath(pathFromEndBallPickupToLaunchZone1);
                    //THIS NEEDS TO CHECK LAUNCH POSITION, NOT START POSITION!
                    if (selectedStartPosition == FrontBackLocation.FRONT) {
                        pathState++;
                    }
                    else {
                        pathState = 7;
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    robot.startSpinTake();
                    shootBalls();
                    pedroMessage = "Going from launch zone to ball pickup 2";
                    follower.followPath(pathFromLaunchZoneToStartBallPickup2);
                    pathState++;
                }
                break;
            case 5:
                if (!follower.isBusy()){
                   pedroMessage = "Ingesting Balls 2";
                   follower.followPath(pathFromStartBallPickupToEndBallPickup2, ballPickupPower, true);
                   pathState++;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    shooterVelocity(2800 * .3);
                    pedroMessage = "Going from ball pickup to launch zone";
                    follower.followPath(pathFromEndBallPickupToLaunchZone2);
                    pathState++;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    shootBalls();
                    robot.stopSpinTake();
                    pedroMessage = "Leaving";
                    follower.followPath(pedroPaths.pathFromNearLaunchZoneToLeave());
                    pathState++;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    done = true;
                }
                break;
        }
        return done;
    }

    /**
     * Set the spike location-dependent PathChains.
     */
    private void setBallSpikeLocationPaths() {
            pathFromLaunchZoneToStartBallPickup1 = pedroPaths.pathFromNearLaunchZoneToGoalStartBallPickup();
            pathFromStartBallPickupToEndBallPickup1 = pedroPaths.pathFromStartToEndGoalBallPickup();
            pathFromEndBallPickupToLaunchZone1 = pedroPaths.pathFromEndGoalPickupToNearLaunchZone();

            pathFromLaunchZoneToStartBallPickup2 = pedroPaths.pathFromNearLaunchZoneToMidStartBallPickup();
            pathFromStartBallPickupToEndBallPickup2 = pedroPaths.pathFromStartToEndMidBallPickup();
            pathFromEndBallPickupToLaunchZone2 = pedroPaths.pathFromEndMidPickupToNearLaunchZone();
    }

    private void setStartPoseAndPath() {
        if (selectedStartPosition == FrontBackLocation.FRONT) {
            startPose = pedroPaths.farStartingPose();
            pathFromStartToLaunchZone = pedroPaths.pathFromWallToNearLaunchZone();
        }
        else {
            startPose = pedroPaths.nearStartingPose();
            pathFromStartToLaunchZone = pedroPaths.pathFromGoalToNearLaunchZone();
        }
    }

    /**
     * Depending on the passed selectedColor, instantiate and return either a
     * RedPedroPathsFrontWall or BluePedroPathsFrontWall.
     * Both implement the AutonomousPathsFrontWall interface.
     * @param selectedColor RED or BLUE
     * @return AutonomousPedroPathsFrontWall
     */
    private GirlsPedroPathsFrontWall createPedroPaths(AllianceColor selectedColor) {
        if (selectedColor == AllianceColor.RED) {
            return new GirlsRedPedroPathsFrontWall(follower, new GirlsBluePedroPathsFrontWall(follower));
        }
        else {
            return new GirlsBluePedroPathsFrontWall(follower);
        }
    }

    private void shootBalls(){
        pedroTelemetry.pathTelemetry("Shooting balls.");
        robot.startSideMotor();
        robot.setTopConveyPower(1);
        sleep(5000);
        robot.setTopConveyPower(0);
        shooterVelocity(0);
    }

    private void shooterVelocity(double velocity) {
        String message = (velocity > 0) ? "Spinning up shooter" : "Spinning down shooter";
        pedroTelemetry.pathTelemetry(message);
        robot.setShooterVelocity(velocity);
    }

    /**
     * Perform pre-start initializations as long as opModeInInit().
     * Get the AllianceColor, and target BallSpikeLocation.
     */
    private void initSetup() {
        while (opModeInInit()) {
            telemetry.addLine("Press Right Bumper to toggle between Red and Blue alliance.");
            telemetry.addData(selectedColor.toString(), " currently selected");
            telemetry.addLine();
            telemetry.addLine("Press Left Bumper to toggle between Near and Far start position.");
            telemetry.addData(selectedStartPosition.toString(), " currently selected");
            telemetry.addLine();
            telemetry.addLine("Press X to toggle between Near and Far Launch Position.");
            telemetry.addData(selectedLaunchPosition.toString()," currently selected");
            telemetry.update();

            selectedColor = AllianceColor.INSTANCE.toggleColor(gamepad1.rightBumperWasPressed(), selectedColor);
            selectedStartPosition = FrontBackLocation.INSTANCE.toggleLocation(gamepad1.leftBumperWasPressed(), selectedStartPosition);
            selectedLaunchPosition = FrontBackLocation.INSTANCE.toggleLocation(gamepad1.xWasPressed(),selectedLaunchPosition);
        }
    }
}
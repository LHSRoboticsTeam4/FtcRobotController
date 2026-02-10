package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.BallSpikeLocation;
import org.firstinspires.ftc.teamcode.GirlsBluePedroPathsFrontWall;
import org.firstinspires.ftc.teamcode.GirlsPedroPathsFrontWall;
import org.firstinspires.ftc.teamcode.GirlsRedPedroPathsFrontWall;
import org.firstinspires.ftc.teamcode.PedroPathConfiguration;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathFollower;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathTelemetry;

@Autonomous(name = "Start from front wall")
public class GirlsAutonomousFrontWallOpMode extends LinearOpMode {
    private PedroPathTelemetry pedroTelemetry;
    private Follower follower;
    private final double ballPickupPower = .4;
    private AllianceColor selectedColor = AllianceColor.RED;
    private BallSpikeLocation ballSpikeLocation = BallSpikeLocation.AUDIENCE_SIDE;
    private GirlsPedroPathsFrontWall pedroPaths;
    private PathChain pathFromLaunchZoneToStartBallPickup;
    private PathChain pathFromStartBallPickupToEndBallPickup;
    private PathChain pathFromEndBallPickupToLaunchZone;

    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(this);

        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);

        follower = pedroPathConfiguration.getFollower();
        initSetup();

        pedroPaths = createPedroPaths(selectedColor);
        setBallSpikeLocationPaths();
        pedroTelemetry = new PedroPathTelemetry(telemetry, follower, selectedColor);
        PedroPathFollower pedroPathFollower = new PedroPathFollower(this, follower, pedroTelemetry, pedroPaths.startingPose());

        waitForStart();

        if (opModeIsActive()) {
            spinTake(2800 * .3);
            pedroPathFollower.followPathChain(pedroPaths.pathFromWallToLaunchZone(), "Going from wall to launch zone");
            shootBalls();
            spinTake(0);
            pedroPathFollower.followPathChain(pathFromLaunchZoneToStartBallPickup, "Going from launch zone to ball pickup");
            ballPickup();
            pedroPathFollower.followPathChain(pathFromStartBallPickupToEndBallPickup, ballPickupPower, "Picking up balls");
            pedroPathFollower.followPathChain(pathFromEndBallPickupToLaunchZone, "Going from ball pickup to launch zone");
            shootBalls();
            pedroPathFollower.followPathChain(pedroPaths.pathFromLaunchZoneToLeave(), "Leaving");
        }
    }

    /**
     * Set the spike location-dependent PathChains.
     */
    private void setBallSpikeLocationPaths() {
        switch (ballSpikeLocation) {
            case AUDIENCE_SIDE:
                pathFromLaunchZoneToStartBallPickup = pedroPaths.pathFromLaunchZoneToAudienceStartBallPickup();
                pathFromStartBallPickupToEndBallPickup = pedroPaths.pathFromStartToEndAudienceBallPickup();
                pathFromEndBallPickupToLaunchZone = pedroPaths.pathFromEndAudiencePickupToLaunchZone();
                break;
            case MIDDLE:
                pathFromLaunchZoneToStartBallPickup = pedroPaths.pathFromLaunchZoneToMidStartBallPickup();
                pathFromStartBallPickupToEndBallPickup = pedroPaths.pathFromStartToEndMidBallPickup();
                pathFromEndBallPickupToLaunchZone = pedroPaths.pathFromEndMidPickupToLaunchZone();
                break;
            case GOAL_SIDE:
                pathFromLaunchZoneToStartBallPickup = pedroPaths.pathFromLaunchZoneToGoalStartBallPickup();
                pathFromStartBallPickupToEndBallPickup = pedroPaths.pathFromStartToEndGoalBallPickup();
                pathFromEndBallPickupToLaunchZone = pedroPaths.pathFromEndGoalPickupToLaunchZone();
                break;
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

    //Emulate shooting balls.
    private void shootBalls(){
        pedroTelemetry.pathTelemetry("Shooting balls.");
        robot.startSpinTake();
        robot.startSideServos();
        robot.setTopConveyPower(1);
        sleep(8000);
        robot.setTopConveyPower(0);
    }

    //Emulate pickup up balls.
    private void ballPickup(){
        pedroTelemetry.pathTelemetry("Ball scooper turned on.");
        sleep(2000);
    }

    private void spinTake(double velocity) {
        String message = (velocity > 0) ? "Spinning up spintake" : "Spinning down spintake";
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
            telemetry.addLine("Press Left Bumper to toggle set ball pickup location");
            telemetry.addData(ballSpikeLocation.toString(), " currently selected");
            telemetry.update();

            selectedColor = AllianceColor.INSTANCE.toggleColor(gamepad1.rightBumperWasPressed(), selectedColor);
            ballSpikeLocation = BallSpikeLocation.INSTANCE.toggleLocation(gamepad1.leftBumperWasPressed(), ballSpikeLocation);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathFollower;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathTelemetry;

@Autonomous(name = "Pedro Far Auto")
@Disabled
public class AutonomousFrontWall extends LinearOpMode {
    private PedroPathTelemetry pedroTelemetry;
    private Follower follower;
    private final double ballPickupPower = .4;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);

        follower = pedroPathConfiguration.getFollower();

        BluePedroPathsFrontWall pedroPaths = new BluePedroPathsFrontWall(follower);
        pedroTelemetry = new PedroPathTelemetry(telemetry, follower, AllianceColor.BLUE);
        PedroPathFollower pedroPathFollower = new PedroPathFollower(this, follower, pedroTelemetry, pedroPaths.startingPose());

        waitForStart();

        if (opModeIsActive()) {
            //robot.setShooterPower(.47);
            robot.setShooterVelocity(2800 * .3);
            pedroPathFollower.followPathChain(pedroPaths.pathFromWallToLaunchZone(), "Going from wall to launch zone");
            robot.startSpinTake();
            robot.startSideServos();
            robot.setTopConveyPower(1);
            sleep(8000);
            robot.setTopConveyPower(0);
            pedroPathFollower.followPathChain(pedroPaths.pathFromLaunchZoneToStartBallPickup(), "Starting ball pickup");
            pedroPathFollower.followPathChain(pedroPaths.pathFromStartToEndBallPickup(), .2, "Ingesting balls");
            pedroPathFollower.followPathChain(pedroPaths.pathFromEndPickupToLaunchZone(), "Going back to launch zone");
            //Sleep while balls are shooting.
            robot.setTopConveyPower(1);
            sleep(8000);
            //Sping things down.
            robot.setShooterPower(0);
            robot.stopSpinTake();
            robot.stopSideServos();
            robot.setTopConveyPower(0);
            pedroPathFollower.followPathChain(pedroPaths.pathFromLaunchZoneToLeave(), "Leaving");
        }
    }
}
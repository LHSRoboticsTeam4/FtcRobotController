package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.teamPedroPathing.FlippablePath;

public class BluePedroPathsFrontWall {
    private final Follower follower;

    private final Pose startingPose = new Pose (56, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose (60, 90, Math.toRadians(140));
    private final Pose startGoalBallPickupPose = new Pose(45, 82, 0);
    private final Pose endGoalBallPickupPose = new Pose(25, 82, 0);
    private final Pose leavePose = new Pose(42, 35, Math.toRadians(180));
    private final Pose startMidBallPickupPickPose = new Pose(45, 60, 0);
    private final Pose endMidBallPickupPickPose = new Pose(25,60,0);
    private final Pose startAudienceBallPickupPickPose = new Pose(45, 35, 0);
    private final Pose endAudienceBallPickupPickPose = new Pose(25,35,0);

    private PathChain pathFromWallToLaunchZone;
    private PathChain pathFromLaunchZoneToStartBallPickup;
    private PathChain pathFromStartToEndBallPickup;
    private PathChain pathFromEndPickupToLaunchZone;
    private PathChain pathFromLaunchZoneToLeave;

    public BluePedroPathsFrontWall(Follower follower) {
        this.follower = follower;
        initPaths();
    }

    private void initPaths() {
        pathFromWallToLaunchZone = buildPathFromWallToLaunchZone();
        pathFromLaunchZoneToStartBallPickup = buildPathFromLaunchZoneToStartBallPickup();
        pathFromStartToEndBallPickup = buildPathFromStartToEndBallPickup();
        pathFromEndPickupToLaunchZone = buildPathFromEndPickupToLaunchZone();
        pathFromLaunchZoneToLeave = buildPathFromLaunchZoneToLeave();
    }

    private PathChain buildPathFromWallToLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(startingPose, shootPose),
            startingPose.getHeading(), shootPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    private PathChain buildPathFromLaunchZoneToStartBallPickup() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(shootPose, startGoalBallPickupPose),
            shootPose.getHeading(), startGoalBallPickupPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    private PathChain buildPathFromStartToEndBallPickup() {
        FlippablePath fPath = FlippablePath.constantHeadingPath(new BezierLine(startGoalBallPickupPose, endGoalBallPickupPose),
            endGoalBallPickupPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    private PathChain buildPathFromEndPickupToLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(endGoalBallPickupPose, shootPose),
            endGoalBallPickupPose.getHeading(), shootPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    private PathChain buildPathFromLaunchZoneToLeave() {
        FlippablePath fPath = FlippablePath.linearHeadingPath((new BezierLine(shootPose, leavePose)),
                shootPose.getHeading(), leavePose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }
    /*******************************************************************************/
    public Pose startingPose() {
        return this.startingPose;
    }

    public PathChain pathFromWallToLaunchZone() {
        return this.pathFromWallToLaunchZone;
    }

    public PathChain pathFromLaunchZoneToStartBallPickup() {
        return this.pathFromLaunchZoneToStartBallPickup;
    }

    public PathChain pathFromStartToEndBallPickup() {
        return this.pathFromStartToEndBallPickup;
    }

    public PathChain pathFromEndPickupToLaunchZone () {
        return this.pathFromEndPickupToLaunchZone;
    }

    public PathChain pathFromLaunchZoneToLeave() {
        return this.pathFromLaunchZoneToLeave;
    }
}

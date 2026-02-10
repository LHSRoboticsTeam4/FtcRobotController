package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.teamPedroPathing.FlippablePath;

public class GirlsBluePedroPathsFrontWall  implements GirlsPedroPathsFrontWall {
    private final Follower follower;

    private final Pose startingPose = new Pose (56, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose (60, 90, Math.toRadians(140));
    private final Pose startGoalBallPickupPose = new Pose(45, 82, 0);
    private final Pose endGoalBallPickupPose = new Pose(25, 82, 0);
    private final Pose leavePose = new Pose(42, 35, Math.toRadians(180));
    private final Pose startMidBallPickupPose = new Pose(45, 60, 0);
    private final Pose endMidBallPickupPose = new Pose(25,60,0);
    private final Pose startAudienceBallPickupPose = new Pose(45, 35, 0);
    private final Pose endAudienceBallPickupPose = new Pose(25,35,0);

    private PathChain pathFromWallToLaunchZone;

    private PathChain pathFromLaunchZoneToGoalStartBallPickup;
    private PathChain pathFromStartToEndGoalBallPickup;
    private PathChain pathFromEndGoalPickupToLaunchZone;

    private PathChain pathFromLaunchZoneToMidStartBallPickup;
    private PathChain pathFromStartToEndMidBallPickup;
    private PathChain pathFromEndMidPickupToLaunchZone;

    private PathChain pathFromLaunchZoneToAudienceStartBallPickup;
    private PathChain pathFromStartToEndAudienceBallPickup;
    private PathChain pathFromEndAudiencePickupToLaunchZone;

    private PathChain pathFromLaunchZoneToLeave;

    public GirlsBluePedroPathsFrontWall(Follower follower) {
        this.follower = follower;
        initPaths();
    }

    private void initPaths() {
        pathFromWallToLaunchZone = buildPathFromWallToLaunchZone();

        pathFromLaunchZoneToGoalStartBallPickup = buildPathFromLaunchZoneToGoalStartBallPickup();
        pathFromStartToEndGoalBallPickup = buildPathFromStartGoalToEndBallPickup();
        pathFromEndGoalPickupToLaunchZone = buildPathFromEndGoalPickupToLaunchZone();

        pathFromLaunchZoneToMidStartBallPickup = buildPathFromLaunchZoneToMidStartBallPickup();
        pathFromStartToEndMidBallPickup = buildPathFromStartMidToEndBallPickup();
        pathFromEndMidPickupToLaunchZone = buildPathFromEndMidPickupToLaunchZone();

        pathFromLaunchZoneToAudienceStartBallPickup = buildPathFromLaunchZoneToAudienceStartBallPickup();
        pathFromStartToEndAudienceBallPickup = buildPathFromStartAudienceToEndBallPickup();
        pathFromEndAudiencePickupToLaunchZone = buildPathFromEndAudiencePickupToLaunchZone();

        pathFromLaunchZoneToLeave = buildPathFromLaunchZoneToLeave();
    }

    private PathChain buildPathFromWallToLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(startingPose, shootPose),
            startingPose.getHeading(), shootPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }
    /************************************************************************************/
    private PathChain buildPathFromLaunchZoneToGoalStartBallPickup() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(shootPose, startGoalBallPickupPose),
            shootPose.getHeading(), startGoalBallPickupPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    private PathChain buildPathFromStartGoalToEndBallPickup() {
        FlippablePath fPath = FlippablePath.constantHeadingPath(new BezierLine(startGoalBallPickupPose, endGoalBallPickupPose),
            endGoalBallPickupPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    private PathChain buildPathFromEndGoalPickupToLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(endGoalBallPickupPose, shootPose),
            endGoalBallPickupPose.getHeading(), shootPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }
    /************************************************************************************/

    private PathChain buildPathFromLaunchZoneToMidStartBallPickup() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(shootPose, startMidBallPickupPose),
                shootPose.getHeading(), startMidBallPickupPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    private PathChain buildPathFromStartMidToEndBallPickup() {
        FlippablePath fPath = FlippablePath.constantHeadingPath(new BezierLine(startMidBallPickupPose, endMidBallPickupPose),
                endMidBallPickupPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    private PathChain buildPathFromEndMidPickupToLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(endMidBallPickupPose, shootPose),
                endMidBallPickupPose.getHeading(), shootPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }
    /************************************************************************************/

    private PathChain buildPathFromLaunchZoneToAudienceStartBallPickup() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(shootPose, startAudienceBallPickupPose),
                shootPose.getHeading(), startAudienceBallPickupPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    private PathChain buildPathFromStartAudienceToEndBallPickup() {
        FlippablePath fPath = FlippablePath.constantHeadingPath(new BezierLine(startAudienceBallPickupPose, endAudienceBallPickupPose),
                endAudienceBallPickupPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    private PathChain buildPathFromEndAudiencePickupToLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(endAudienceBallPickupPose, shootPose),
                endAudienceBallPickupPose.getHeading(), shootPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    /************************************************************************************/

    private PathChain buildPathFromLaunchZoneToLeave() {
        FlippablePath fPath = FlippablePath.linearHeadingPath((new BezierLine(shootPose, leavePose)),
                shootPose.getHeading(), leavePose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }
    /*******************************************************************************/

    @Override
    public Pose startingPose() {
        return this.startingPose;
    }

    @Override
    public PathChain pathFromWallToLaunchZone() {
        return this.pathFromWallToLaunchZone;
    }
    /*******************************************************************************/

    @Override
    public PathChain pathFromLaunchZoneToGoalStartBallPickup() {
        return this.pathFromLaunchZoneToGoalStartBallPickup;
    }

    @Override
    public PathChain pathFromStartToEndGoalBallPickup() {
        return this.pathFromStartToEndGoalBallPickup;
    }

    @Override
    public PathChain pathFromEndGoalPickupToLaunchZone () {
        return this.pathFromEndGoalPickupToLaunchZone;
    }
    /*******************************************************************************/

    @Override
    public PathChain pathFromLaunchZoneToMidStartBallPickup() {
        return this.pathFromLaunchZoneToMidStartBallPickup;
    }

    @Override
    public PathChain pathFromStartToEndMidBallPickup() {
        return this.pathFromStartToEndMidBallPickup;
    }

    @Override
    public PathChain pathFromEndMidPickupToLaunchZone () {
        return this.pathFromEndMidPickupToLaunchZone;
    }
    /*******************************************************************************/

    @Override
    public PathChain pathFromLaunchZoneToAudienceStartBallPickup() {
        return this.pathFromLaunchZoneToAudienceStartBallPickup;
    }

    @Override
    public PathChain pathFromStartToEndAudienceBallPickup() {
        return this.pathFromStartToEndAudienceBallPickup;
    }

    @Override
    public PathChain pathFromEndAudiencePickupToLaunchZone () {
        return this.pathFromEndAudiencePickupToLaunchZone;
    }
    /*******************************************************************************/

    @Override
    public PathChain pathFromLaunchZoneToLeave() {
        return this.pathFromLaunchZoneToLeave;
    }
}

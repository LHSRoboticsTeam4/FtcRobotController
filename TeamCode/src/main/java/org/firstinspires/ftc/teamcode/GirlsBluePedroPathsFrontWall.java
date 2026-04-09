package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.teamPedroPathing.FlippablePath;

public class GirlsBluePedroPathsFrontWall  implements GirlsPedroPathsFrontWall {
    private final Follower follower;

    private final Pose farStartingPose = new Pose (56, 8, Math.toRadians(90));

    private final Pose nearStartingPose = new Pose (20.5,124.4, Math.toRadians(144));
    private final Pose nearLaunchPose = new Pose (60, 90, Math.toRadians(140));
    private final Pose farLaunchPose = new Pose(56, 14, Math.toRadians(110));

    private final Pose parkPose = new Pose (107,31, 0);
    private final Pose startGoalBallPickupPose = new Pose(45, 82, 0);
    private final Pose endGoalBallPickupPose = new Pose(25, 82, 0);
    private final Pose leavePose = new Pose(42, 35, Math.toRadians(180));
    private final Pose startMidBallPickupPose = new Pose(45, 60, 0);
    private final Pose endMidBallPickupPose = new Pose(25,60,0);
    private final Pose startAudienceBallPickupPose = new Pose(45, 35, 0);
    private final Pose endAudienceBallPickupPose = new Pose(25,35,0);

    private PathChain pathFromWallToNearLaunchZone;
    private PathChain pathFromWallToFarLaunchZone;

    private PathChain pathFromGoalToLaunchZone;

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
    private PathChain pathFromFarLaunchZoneToLeave;

    private PathChain pathFromEndAudiencePickupToFarLaunchZone;
    private PathChain pathFromFarLaunchZoneToAudienceStartBallPickup;
    public GirlsBluePedroPathsFrontWall(Follower follower) {
        this.follower = follower;
        initPaths();
    }

    private void initPaths() {
        pathFromWallToNearLaunchZone = buildPathFromWallToNearLaunchZone();
        pathFromWallToFarLaunchZone = buildPathFromWallToFarLaunchZone();

        pathFromGoalToLaunchZone = buildPathFromGoalToNearLaunchZone();

        pathFromLaunchZoneToGoalStartBallPickup = buildPathNearFromLaunchZoneToGoalStartBallPickup();
        pathFromStartToEndGoalBallPickup = buildPathFromStartGoalToEndBallPickup();
        pathFromEndGoalPickupToLaunchZone = buildPathFromEndGoalPickupToNearLaunchZone();

        pathFromLaunchZoneToMidStartBallPickup = buildPathFromLaunchZoneToMidStartBallPickup();
        pathFromStartToEndMidBallPickup = buildPathFromStartMidToEndBallPickup();
        pathFromEndMidPickupToLaunchZone = buildPathFromEndMidPickupToNearLaunchZone();

        pathFromLaunchZoneToAudienceStartBallPickup = buildPathFromNearLaunchZoneToAudienceStartBallPickup();
        pathFromFarLaunchZoneToAudienceStartBallPickup = buildPathFromFarLaunchZoneToAudienceStartBallPickup();

        pathFromStartToEndAudienceBallPickup = buildPathFromStartAudienceToEndBallPickup();
        pathFromEndAudiencePickupToLaunchZone = buildPathFromEndAudiencePickupToNearLaunchZone();
        pathFromEndAudiencePickupToFarLaunchZone = buildPathFromEndAudiencePickupToFarLaunchZone();

        pathFromLaunchZoneToLeave = buildPathFromNearLaunchZoneToLeave();
        pathFromFarLaunchZoneToLeave = buildPathFromFarLaunchZoneToLeave();
    }

    private PathChain buildPathFromWallToNearLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(farStartingPose, nearLaunchPose),
            farStartingPose.getHeading(), nearLaunchPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    private PathChain buildPathFromFarLaunchZoneToAudienceStartBallPickup() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(farLaunchPose, startAudienceBallPickupPose),
                farLaunchPose.getHeading(), startAudienceBallPickupPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }
    private PathChain buildPathFromWallToFarLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(farStartingPose, farLaunchPose),
                farStartingPose.getHeading(), farLaunchPose.getHeading());

        return follower.pathBuilder()
            .addPath(fPath)
            .build();
    }

    private PathChain buildPathFromGoalToNearLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(nearStartingPose, nearLaunchPose),
                farStartingPose.getHeading(), nearLaunchPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }
    private PathChain buildPathFromEndAudiencePickupToFarLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(endAudienceBallPickupPose, farLaunchPose),
                endAudienceBallPickupPose.getHeading(), farLaunchPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }
    private PathChain buildPathFromFarLaunchZoneToLeave() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(farLaunchPose, leavePose),
                farLaunchPose.getHeading(), leavePose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();

    }
    /************************************************************************************/
    private PathChain buildPathNearFromLaunchZoneToGoalStartBallPickup() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(nearLaunchPose, startGoalBallPickupPose),
            nearLaunchPose.getHeading(), startGoalBallPickupPose.getHeading());

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

    private PathChain buildPathFromEndGoalPickupToNearLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(endGoalBallPickupPose, nearLaunchPose),
            endGoalBallPickupPose.getHeading(), nearLaunchPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }
    /************************************************************************************/

    private PathChain buildPathFromLaunchZoneToMidStartBallPickup() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(nearLaunchPose, startMidBallPickupPose),
                nearLaunchPose.getHeading(), startMidBallPickupPose.getHeading());

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

    private PathChain buildPathFromEndMidPickupToNearLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(endMidBallPickupPose, nearLaunchPose),
                endMidBallPickupPose.getHeading(), nearLaunchPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }
    /************************************************************************************/

    private PathChain buildPathFromNearLaunchZoneToAudienceStartBallPickup() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(nearLaunchPose, startAudienceBallPickupPose),
                nearLaunchPose.getHeading(), startAudienceBallPickupPose.getHeading());

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

    private PathChain buildPathFromEndAudiencePickupToNearLaunchZone() {
        FlippablePath fPath = FlippablePath.linearHeadingPath(new BezierLine(endAudienceBallPickupPose, nearLaunchPose),
                endAudienceBallPickupPose.getHeading(), nearLaunchPose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }

    /************************************************************************************/

    private PathChain buildPathFromNearLaunchZoneToLeave() {
        FlippablePath fPath = FlippablePath.linearHeadingPath((new BezierLine(nearLaunchPose, leavePose)),
                nearLaunchPose.getHeading(), leavePose.getHeading());

        return follower.pathBuilder()
                .addPath(fPath)
                .build();
    }
    /*******************************************************************************/

    @Override
    public Pose farStartingPose() {
        return this.farStartingPose;
    }

    @Override
    public Pose nearStartingPose() {return this.nearStartingPose; }

    @Override
    public Pose parkPose() {return this.parkPose; }

    @Override
    public PathChain pathFromWallToNearLaunchZone() {
        return this.pathFromWallToNearLaunchZone;
    }

    public PathChain pathFromWallToFarLaunchZone() {
        return this.pathFromWallToFarLaunchZone;
    }
    /*******************************************************************************/

    @Override
    public PathChain pathFromGoalToNearLaunchZone() {return this.pathFromGoalToLaunchZone;}

    @Override
    public PathChain pathFromNearLaunchZoneToGoalStartBallPickup() {
        return this.pathFromLaunchZoneToGoalStartBallPickup;
    }
    public PathChain pathFromFarLaunchZoneToAudienceStartBallPickup() {
        return this.pathFromFarLaunchZoneToAudienceStartBallPickup;
    }
    @Override
    public PathChain pathFromStartToEndGoalBallPickup() {
        return this.pathFromStartToEndGoalBallPickup;
    }

    @Override
    public PathChain pathFromEndGoalPickupToNearLaunchZone() {
        return this.pathFromEndGoalPickupToLaunchZone;
    }

    /*******************************************************************************/
    @Override
    public PathChain pathFromNearLaunchZoneToMidStartBallPickup() {
        return this.pathFromLaunchZoneToMidStartBallPickup;
    }

    @Override
    public PathChain pathFromStartToEndMidBallPickup() {
        return this.pathFromStartToEndMidBallPickup;
    }

    @Override
    public PathChain pathFromEndMidPickupToNearLaunchZone() {
        return this.pathFromEndMidPickupToLaunchZone;
    }
    /*******************************************************************************/

    @Override
    public PathChain pathFromNearLaunchZoneToAudienceStartBallPickup() {
        return this.pathFromLaunchZoneToAudienceStartBallPickup;
    }

    @Override
    public PathChain pathFromStartToEndAudienceBallPickup() {
        return this.pathFromStartToEndAudienceBallPickup;
    }

    @Override
    public PathChain pathFromEndAudiencePickupToNearLaunchZone() {
        return this.pathFromEndAudiencePickupToLaunchZone;
    }
    public PathChain pathFromEndAudiencePickupToFarLaunchZone() {
        return this.pathFromEndAudiencePickupToFarLaunchZone;
    }
    /*******************************************************************************/

    @Override
    public PathChain pathFromNearLaunchZoneToLeave() {
        return this.pathFromLaunchZoneToLeave;
    }

    public PathChain pathFromFarLaunchZoneToLeave() {
        return this.pathFromFarLaunchZoneToLeave;
    }
}

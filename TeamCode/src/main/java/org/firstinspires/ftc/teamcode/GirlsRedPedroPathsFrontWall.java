package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.teamPedroPathing.FlippablePath;
import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathFlipper;

public class GirlsRedPedroPathsFrontWall implements GirlsPedroPathsFrontWall {
    private final Follower follower;

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

    private PedroPathFlipper pathFlipper;
    private GirlsPedroPathsFrontWall girlsBluePedroPathsFrontWall;

    public GirlsRedPedroPathsFrontWall(Follower follower, GirlsPedroPathsFrontWall girlsBluePedroPathsFrontWall) {
        this.follower = follower;
        this.pathFlipper = new PedroPathFlipper(follower);
        this.girlsBluePedroPathsFrontWall = girlsBluePedroPathsFrontWall;
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
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromWallToLaunchZone());
    }
    /************************************************************************************/
    private PathChain buildPathFromLaunchZoneToGoalStartBallPickup() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromLaunchZoneToGoalStartBallPickup());
    }

    private PathChain buildPathFromStartGoalToEndBallPickup() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromStartToEndGoalBallPickup());
    }

    private PathChain buildPathFromEndGoalPickupToLaunchZone() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromEndGoalPickupToLaunchZone());
    }

    /************************************************************************************/

    private PathChain buildPathFromLaunchZoneToMidStartBallPickup() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromLaunchZoneToMidStartBallPickup());
    }

    private PathChain buildPathFromStartMidToEndBallPickup() {
        return pathFlipper.flipPathChain((girlsBluePedroPathsFrontWall.pathFromStartToEndMidBallPickup()));
    }

    private PathChain buildPathFromEndMidPickupToLaunchZone() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromEndMidPickupToLaunchZone());
    }
    /************************************************************************************/

    private PathChain buildPathFromLaunchZoneToAudienceStartBallPickup() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromLaunchZoneToAudienceStartBallPickup());
    }

    private PathChain buildPathFromStartAudienceToEndBallPickup() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromStartToEndAudienceBallPickup());
    }

    private PathChain buildPathFromEndAudiencePickupToLaunchZone() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromEndAudiencePickupToLaunchZone());
    }

    /************************************************************************************/
    private PathChain buildPathFromLaunchZoneToLeave() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromLaunchZoneToLeave());
    }

    /************************************************************************************/
    @Override
    public Pose startingPose() {
        return this.pathFromWallToLaunchZone.firstPath().getFirstControlPoint();
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
    public PathChain pathFromEndGoalPickupToLaunchZone() {
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
    public PathChain pathFromEndMidPickupToLaunchZone() {
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
    public PathChain pathFromEndAudiencePickupToLaunchZone() {
        return this.pathFromEndAudiencePickupToLaunchZone;
    }
    /*******************************************************************************/

    @Override
    public PathChain pathFromLaunchZoneToLeave() {
        return this.pathFromLaunchZoneToLeave;
    }
}

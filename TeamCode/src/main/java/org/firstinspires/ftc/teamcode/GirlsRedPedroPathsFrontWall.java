package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathFlipper;

public class GirlsRedPedroPathsFrontWall implements GirlsPedroPathsFrontWall {
    private final Follower follower;

    private PathChain pathFromWallToNearLaunchZone;

    private PathChain pathFromGoalToNearLaunchZone;

    private PathChain pathFromNearLaunchZoneToGoalStartBallPickup;
    private PathChain pathFromStartToEndGoalBallPickup;
    private PathChain pathFromEndGoalPickupToNearLaunchZone;

    private PathChain pathFromNearLaunchZoneToMidStartBallPickup;
    private PathChain pathFromStartToEndMidBallPickup;
    private PathChain pathFromEndMidPickupToNearLaunchZone;

    private PathChain pathFromNearLaunchZoneToAudienceStartBallPickup;
    private PathChain pathFromStartToEndAudienceBallPickup;
    private PathChain pathFromEndAudiencePickupToNearLaunchZone;

    private PathChain pathFromNearLaunchZoneToLeave;

    private PathChain pathFromFarLaunchZoneToAudienceStartBallPickup;
    private PathChain pathFromEndAudiencePickupToFarLaunchZone;
    private PathChain pathFromFarLaunchZoneToLeave;
    private PathChain pathFromWallToFarLaunchZone;


    private PedroPathFlipper pathFlipper;
    private GirlsPedroPathsFrontWall girlsBluePedroPathsFrontWall;

    public GirlsRedPedroPathsFrontWall(Follower follower, GirlsPedroPathsFrontWall girlsBluePedroPathsFrontWall) {
        this.follower = follower;
        this.pathFlipper = new PedroPathFlipper(follower);
        this.girlsBluePedroPathsFrontWall = girlsBluePedroPathsFrontWall;
        initPaths();
    }

    private void initPaths() {
        pathFromWallToNearLaunchZone = buildPathFromWallToLaunchZone();
        pathFromGoalToNearLaunchZone = buildPathFromGoalToLaunchZone();

        pathFromNearLaunchZoneToGoalStartBallPickup = buildPathFromLaunchZoneToGoalStartBallPickup();
        pathFromStartToEndGoalBallPickup = buildPathFromStartGoalToEndBallPickup();
        pathFromEndGoalPickupToNearLaunchZone = buildPathFromEndGoalPickupToLaunchZone();

        pathFromNearLaunchZoneToMidStartBallPickup = buildPathFromLaunchZoneToMidStartBallPickup();
        pathFromStartToEndMidBallPickup = buildPathFromStartMidToEndBallPickup();
        pathFromEndMidPickupToNearLaunchZone = buildPathFromEndMidPickupToLaunchZone();

        pathFromNearLaunchZoneToAudienceStartBallPickup = buildPathFromLaunchZoneToAudienceStartBallPickup();
        pathFromStartToEndAudienceBallPickup = buildPathFromStartAudienceToEndBallPickup();
        pathFromEndAudiencePickupToNearLaunchZone = buildPathFromEndAudiencePickupToLaunchZone();

        pathFromNearLaunchZoneToLeave = buildPathFromLaunchZoneToLeave();

        pathFromFarLaunchZoneToAudienceStartBallPickup = buildPathFromFarLaunchZoneToAudienceStartBallPickup();
        pathFromEndAudiencePickupToFarLaunchZone = buildPathFromEndAudiencePickupToFarLaunchZone();
        pathFromFarLaunchZoneToLeave = buildPathFromFarLaunchZoneToLeave();
        pathFromWallToFarLaunchZone = buildPathFromWallToFarLaunchZone();

    }

   private  PathChain buildPathFromFarLaunchZoneToAudienceStartBallPickup() {
       return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromFarLaunchZoneToAudienceStartBallPickup());
   }
   private PathChain buildPathFromEndAudiencePickupToFarLaunchZone() {
       return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromEndAudiencePickupToFarLaunchZone());
   }
   private PathChain buildPathFromFarLaunchZoneToLeave(){
       return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromFarLaunchZoneToLeave());
   }
   private PathChain buildPathFromWallToFarLaunchZone(){
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromWallToFarLaunchZone());
   }

    private PathChain buildPathFromWallToLaunchZone() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromWallToNearLaunchZone());
    }

    private PathChain buildPathFromGoalToLaunchZone() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromGoalToNearLaunchZone());
    }
    /************************************************************************************/
    private PathChain buildPathFromLaunchZoneToGoalStartBallPickup() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromNearLaunchZoneToGoalStartBallPickup());
    }

    private PathChain buildPathFromStartGoalToEndBallPickup() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromStartToEndGoalBallPickup());
    }

    private PathChain buildPathFromEndGoalPickupToLaunchZone() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromEndGoalPickupToNearLaunchZone());
    }

    /************************************************************************************/

    private PathChain buildPathFromLaunchZoneToMidStartBallPickup() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromNearLaunchZoneToMidStartBallPickup());
    }

    private PathChain buildPathFromStartMidToEndBallPickup() {
        return pathFlipper.flipPathChain((girlsBluePedroPathsFrontWall.pathFromStartToEndMidBallPickup()));
    }

    private PathChain buildPathFromEndMidPickupToLaunchZone() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromEndMidPickupToNearLaunchZone());
    }
    /************************************************************************************/

    private PathChain buildPathFromLaunchZoneToAudienceStartBallPickup() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromNearLaunchZoneToAudienceStartBallPickup());
    }

    private PathChain buildPathFromStartAudienceToEndBallPickup() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromStartToEndAudienceBallPickup());
    }

    private PathChain buildPathFromEndAudiencePickupToLaunchZone() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromEndAudiencePickupToNearLaunchZone());
    }

    /************************************************************************************/
    private PathChain buildPathFromLaunchZoneToLeave() {
        return pathFlipper.flipPathChain(girlsBluePedroPathsFrontWall.pathFromNearLaunchZoneToLeave());
    }

    /************************************************************************************/
    @Override
    public Pose farStartingPose() {
        Pose bluePose = girlsBluePedroPathsFrontWall.farStartingPose();
        return new Pose(144 - bluePose.getX(), bluePose.getY(),
                MathFunctions.normalizeAngle(Math.PI - bluePose.getHeading()));
    }
    @Override
    public Pose nearStartingPose() {
        Pose bluePose = girlsBluePedroPathsFrontWall.nearStartingPose();
        return new Pose(144 - bluePose.getX(), bluePose.getY(),
                MathFunctions.normalizeAngle(Math.PI - bluePose.getHeading()));
    }

    @Override
    public Pose parkPose() {
        Pose bluePose = girlsBluePedroPathsFrontWall.parkPose();

        return new Pose(
                144 - bluePose.getX(), bluePose.getY(),
                MathFunctions.normalizeAngle(Math.PI - bluePose.getHeading())
        );
    }

    @Override
    public PathChain pathFromWallToNearLaunchZone() {
        return this.pathFromWallToNearLaunchZone;
    }
    /*******************************************************************************/

    @Override
    public PathChain pathFromGoalToNearLaunchZone() { return this.pathFromGoalToNearLaunchZone; }

    @Override
    public PathChain pathFromNearLaunchZoneToGoalStartBallPickup() {
        return this.pathFromNearLaunchZoneToGoalStartBallPickup;
    }

    @Override
    public PathChain pathFromStartToEndGoalBallPickup() {
        return this.pathFromStartToEndGoalBallPickup;
    }

    @Override
    public PathChain pathFromEndGoalPickupToNearLaunchZone() {
        return this.pathFromEndGoalPickupToNearLaunchZone;
    }
    /*******************************************************************************/

    @Override
    public PathChain pathFromNearLaunchZoneToMidStartBallPickup() {
        return this.pathFromNearLaunchZoneToMidStartBallPickup;
    }

    @Override
    public PathChain pathFromStartToEndMidBallPickup() {
        return this.pathFromStartToEndMidBallPickup;
    }

    @Override
    public PathChain pathFromEndMidPickupToNearLaunchZone() {
        return this.pathFromEndMidPickupToNearLaunchZone;
    }
    /*******************************************************************************/

    @Override
    public PathChain pathFromNearLaunchZoneToAudienceStartBallPickup() {
        return this.pathFromNearLaunchZoneToAudienceStartBallPickup;
    }

    @Override
    public PathChain pathFromStartToEndAudienceBallPickup() {
        return this.pathFromStartToEndAudienceBallPickup;
    }

    @Override
    public PathChain pathFromEndAudiencePickupToNearLaunchZone() {
        return this.pathFromEndAudiencePickupToNearLaunchZone;
    }
    /*******************************************************************************/

    @Override
    public PathChain pathFromNearLaunchZoneToLeave() {
        return this.pathFromNearLaunchZoneToLeave;
    }
    public PathChain pathFromFarLaunchZoneToAudienceStartBallPickup() {
        return this.pathFromFarLaunchZoneToAudienceStartBallPickup;
    }
    public PathChain pathFromEndAudiencePickupToFarLaunchZone() {
        return this.pathFromEndAudiencePickupToFarLaunchZone;
    }
    public PathChain pathFromFarLaunchZoneToLeave() {
        return this.pathFromFarLaunchZoneToLeave;
    }
    public PathChain pathFromWallToFarLaunchZone() {
        return this.pathFromWallToFarLaunchZone;
    }
}

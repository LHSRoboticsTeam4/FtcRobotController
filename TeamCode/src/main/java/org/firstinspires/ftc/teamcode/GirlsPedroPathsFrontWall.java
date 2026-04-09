package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public interface GirlsPedroPathsFrontWall {
    Pose farStartingPose();

    Pose nearStartingPose();

    Pose parkPose();

    PathChain pathFromWallToNearLaunchZone();
    PathChain pathFromWallToFarLaunchZone();

    PathChain pathFromGoalToNearLaunchZone();

    PathChain pathFromNearLaunchZoneToGoalStartBallPickup();

    PathChain pathFromStartToEndGoalBallPickup();

    PathChain pathFromEndGoalPickupToNearLaunchZone();

    PathChain pathFromNearLaunchZoneToMidStartBallPickup();

    PathChain pathFromStartToEndMidBallPickup();


    PathChain pathFromEndMidPickupToNearLaunchZone();
    PathChain pathFromNearLaunchZoneToAudienceStartBallPickup();
    PathChain pathFromFarLaunchZoneToAudienceStartBallPickup();

    PathChain pathFromStartToEndAudienceBallPickup();

    PathChain pathFromEndAudiencePickupToNearLaunchZone();
    PathChain pathFromEndAudiencePickupToFarLaunchZone();

    PathChain pathFromNearLaunchZoneToLeave();
    PathChain pathFromFarLaunchZoneToLeave();
}

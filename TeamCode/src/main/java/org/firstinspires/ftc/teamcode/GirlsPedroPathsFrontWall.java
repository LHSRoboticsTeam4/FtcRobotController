package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public interface GirlsPedroPathsFrontWall {
    Pose startingPose();

    PathChain pathFromWallToLaunchZone();

    PathChain pathFromLaunchZoneToGoalStartBallPickup();

    PathChain pathFromStartToEndGoalBallPickup();

    PathChain pathFromEndGoalPickupToLaunchZone();

    PathChain pathFromLaunchZoneToMidStartBallPickup();

    PathChain pathFromStartToEndMidBallPickup();

    PathChain pathFromEndMidPickupToLaunchZone();

    PathChain pathFromLaunchZoneToAudienceStartBallPickup();

    PathChain pathFromStartToEndAudienceBallPickup();

    PathChain pathFromEndAudiencePickupToLaunchZone();

    PathChain pathFromLaunchZoneToLeave();
}

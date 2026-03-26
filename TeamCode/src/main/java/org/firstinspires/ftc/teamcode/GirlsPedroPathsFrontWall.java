package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public interface GirlsPedroPathsFrontWall {
    Pose farStartingPose();

    Pose nearStartingPose();

    Pose parkPose();

    PathChain pathFromWallToLaunchZone();

    PathChain pathFromGoalToLaunchZone();

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

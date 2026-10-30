package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;
import org.lhssa.ftc.teamcode.pedroPathing.PedroMotion;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPathData;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPather;

@Autonomous(name = "BioBuzz Auto")
public class BioBuzzAuto extends LinearOpMode {

    private AllianceColor allianceColor;
    private int currentStep = 0;
    private Follower follower;
    private PedroPather pedroPather;
    private PedroMotion pedroMotion;

    private Pose startPose = new Pose(83, 130, Math.toRadians(90));


    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration(this);
        follower = pedroPathConfiguration.getFollower();

        initSetup();
        pedroPather = new PedroPather(org.lhssa.ftc.teamcode.pedroPathing.AllianceColor.BLUE, allianceColor,
                PedroPather::FlipRoutineAlliancesOnOppositeSides);
        pedroMotion =new PedroMotion(follower);

        follower.setPose(startPose);
        telemetry.addLine("Starting Pose:");
        telemetry.addData("x", startPose.x());
        telemetry.addData("y", startPose.y());
        telemetry.addData("heading", Math.toDegrees(startPose.heading()));
        telemetry.addData("SpinTake", robot.getIsSpinTakeRunning() ? "Running" : "Stopped");
        telemetry.update();

//        blueDrive(startPose);

        while (opModeIsActive()) {
            follower.update();
            performAction();
        }
    }

    private void performAction() {
        PedroPathData path;
        switch (currentStep) {
            case 0:
                path = pedroPather.pathBetween(startPose, new Pose(135, 115, Math.toRadians(-90)));
                pedroMotion.goPath(path);
                if (pedroMotion.isPathComplete()) {
                    currentStep++;
                }
                break;
            case 1:
                //stuff here
                break;
        }
    }

    private void initSetup() {
        while (opModeInInit()) {
            telemetry.addLine("Press Right Bumper to toggle between Red and Blue alliance.");
            telemetry.addData(allianceColor.toString(), " currently selected");
            telemetry.update();
            allianceColor = AllianceColor.INSTANCE.toggleColor(gamepad1.rightBumperWasPressed(), allianceColor);
        }
    }

    private void blueDrive(Pose startPose) {
        // TODO Go to pollen pick up

        Pose alignForPickup = new Pose(135, 115, Math.toRadians(-90));
        Pose pollenPickUp = new Pose(135, 135, Math.toRadians(90));

        follower.follow(Paths.line(startPose, alignForPickup).linear(startPose, alignForPickup));
        follower.follow(Paths.line(alignForPickup, pollenPickUp).reverseTangent());


        // TODO go to park
        Pose alignForPark = new Pose(115, 97);
        Pose park = new Pose(130, 46);

        follower.follow(Paths.line(pollenPickUp /*change to shootArea later*/, alignForPark).tangent());
        follower.follow(Paths.line(alignForPark, park).tangent());
    }
    private void blueDrive2(Pose startPose) {
        Pose parkZone = new Pose(135, 36, Math.PI * 3 / 2);
        follower.follow(Paths.line(startPose, parkZone).linear(startPose,parkZone));
    }
    private void redDrive2(Pose startPose){
        Pose parkZone = new Pose(6, 105, Math.PI * 3 / 2);
        follower.follow(Paths.line(startPose, parkZone).linear(startPose,parkZone));

    }
    
    private void redDrive(Pose startPose) {
        //To goto pollen do pick up
        Pose redAlignForPickup = new Pose(56, 36, -Math.PI / 2);
       Pose redPollenPickUp = new Pose(8, 8, 0);


       follower.follow(Paths.line(startPose, redAlignForPickup).linear(startPose, redAlignForPickup));
       follower.follow(Paths.line(redAlignForPickup, redPollenPickUp).reverseTangent());


       // TODO go to park
       Pose redAlignForPark = new Pose(22, 49);
       Pose park2 = new Pose(6, 100);

       follower.follow(Paths.line(redPollenPickUp /*change to shootArea later*/, redAlignForPark).tangent());
       follower.follow(Paths.line(redAlignForPark, park2).tangent());
   }

    }


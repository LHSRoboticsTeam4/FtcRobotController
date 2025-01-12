package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name = "AutoBucket")
@Disabled()
public class AutoBucketHOLD extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    RobotWheels wheels = new RobotWheels(this, robot);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        wheels.init();
        waitForStart();
        if (opModeIsActive()) {
            //wheels.autoDriveRobot(35, 35);
            //wheels.autoDriveRobot(23, -23);

            robot.intakeArmDown();
            sleep(1500);

            robot.intake();
            sleep(5000);
//            if (robot.isRightColor(SpikeColor.BLUE)) {
//                robot.stopIntake();
//                robot.intakeArmMid();
//                wheels.autoDriveRobot(-15, 15);
//               // wheels.autoDriveRobot(30, 30);
//            }

        }
    }
}

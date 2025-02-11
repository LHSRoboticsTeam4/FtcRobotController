package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoBucket")
public class AutoBucket extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    RobotWheels wheels = new RobotWheels(this, robot);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        wheels.init();
        waitForStart();
        if (opModeIsActive()) {
            wheels.autoDriveRobot(-13, -13);
            wheels.autoDriveRobot(10,-10);
            wheels.autoDriveRobot(-2, -2);
            //sleep(3000);
            robot.intakeArmMid();
            robot.boxServoDown();
            sleep(1000);
            robot.moveSliderUpAuto();
            robot.boxServoUp();
            robot.boxServoDown();
            robot.moveSliderDownAuto();
            robot.intakeArmMid();
            wheels.autoDriveRobot(13,13);
            wheels.autoDriveRobot(17, -17);
            wheels.autoDriveRobot(50,50);
            wheels.autoDriveRobot(19, -19);
            wheels.autoDriveRobot(-11, -11);
            robot.intakeArmMid();
            robot.boxServoUp();
            robot.intakeArmUp();
        }
    }
}

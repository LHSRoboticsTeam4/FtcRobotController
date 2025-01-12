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
            sleep(500);
            robot.intakeArmMid();
            robot.moveSliderUp();
            robot.boxServoUp();
            robot.boxServoDown();
            robot.moveSliderDown();
            sleep(2000);
        }
    }
}

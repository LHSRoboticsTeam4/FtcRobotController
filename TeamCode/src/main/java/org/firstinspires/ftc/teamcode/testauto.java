package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name = "test")
public class testauto extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    RobotWheels wheels = new RobotWheels(this, robot);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        wheels.init();
        waitForStart();
        wheels.autoDriveRobot(35, 35);
    }
}

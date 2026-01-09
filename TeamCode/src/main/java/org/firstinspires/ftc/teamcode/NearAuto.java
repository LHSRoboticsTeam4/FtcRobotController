package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Near Auto")
public class NearAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    RobotWheels wheels = new RobotWheels(this, robot);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        wheels.init();
        waitForStart();
        if (opModeIsActive()) {
            robot.setShooterPower(.625);
            //Give time for shooter motors to come up to speed.
            sleep(2000);
            robot.startSpinTake();
            robot.startSideServos();

            wheels.autoDriveRobot(15, 15);

            robot.setTopConveyPower(1);

            //Wait to move robot after shooting balls.
            sleep(20000);
        }
    }
}
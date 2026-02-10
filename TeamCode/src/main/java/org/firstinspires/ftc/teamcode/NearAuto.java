package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Near Auto")
@Disabled
public class NearAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    RobotWheels wheels = new RobotWheels(this, robot);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        wheels.init();
        waitForStart();
        if (opModeIsActive()) {
            robot.setShooterPower(.75);
            //Give time for shooter motors to come up to speed.
            wheels.autoDriveRobot(15, 15);
            //sleep(2000);
            robot.startSpinTake();
            robot.startSideServos();


            robot.setTopConveyPower(1);

            //Sleep while balls are shooting.
            sleep(10000);

            //Sping things down.
            robot.setShooterPower(0);
            robot.stopSpinTake();
            robot.stopSideServos();
            robot.setTopConveyPower(0);

            //Leave.
            wheels.autoDriveRobot(-15,15);
            wheels.autoDriveRobot(-12,-12);

        }
    }
}
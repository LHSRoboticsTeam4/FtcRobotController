package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Far Auto")
public class FarAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    RobotWheels wheels = new RobotWheels(this, robot);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        wheels.init();
        waitForStart();
        if (opModeIsActive()) {
            //Spin up while moving robot.
            robot.setShooterPower(1);

            wheels.autoDriveRobot(-55, -55);
            wheels.autoDriveRobot(-9,9);

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

            //Park.
            wheels.autoDriveRobot(-15,15);
            wheels.autoDriveRobot(-8,-8);
        }
    }
}
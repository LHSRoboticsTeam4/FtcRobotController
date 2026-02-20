package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Far Shoot then Leave")
public class FarAuto extends LinearOpMode {
    RobotHardware robot;
    RobotWheels wheels;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(this);
        wheels = new RobotWheels(this, robot);
        wheels.init();
        waitForStart();
        if (opModeIsActive()) {
            robot.setShooterVelocity(2800 * .38);
            sleep(2000);
            //wheels.autoDriveRobot(-55, -55);
            //wheels.autoDriveRobot(-8,8);

            //Shoot!
            robot.startSpinTake();
            robot.startSideMotor();
            robot.setTopConveyPower(1);

            //Sleep while balls are shooting.
            sleep(7000);

            //Sping things down.
            robot.setShooterPower(0);
            robot.stopSpinTake();
            robot.stopSideMotor();
            robot.setTopConveyPower(0);

            //Park.
            wheels.autoDriveRobot(-10, -10);
/*
            wheels.autoDriveRobot(-15,15);
            wheels.autoDriveRobot(-12,-12);
*/
        }
    }
}
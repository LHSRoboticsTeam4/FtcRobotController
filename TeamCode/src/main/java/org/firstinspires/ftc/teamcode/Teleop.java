package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Manually control robot", group = "Linear OpMode")
public class Teleop extends LinearOpMode {

    // DO NOT initialize hardware here
    private RobotHardware robot;
    private RobotWheels robotwheels;

    private int shooterState = 0;
    private String shooterDistance = "None";

    @Override
    public void runOpMode() {

        // Initialize hardware INSIDE runOpMode
        robot = new RobotHardware(this);
        robotwheels = new RobotWheels(this, robot);

        robot.init();
        robotwheels.init();

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            robot.startSpinTake();
            robot.startSideServos();
        }

        while (opModeIsActive()) {

            robotwheels.manuallyDriveRobot(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );

            if (gamepad1.x) {
                toggleShooterDistance();
                sleep(500); // debounce
            }

            if (gamepad1.y) {
                robot.setTopConveyPower(1);
            } else {
                robot.setTopConveyPower(0);
            }

            telemetry.update();
        }
    }

    private void toggleShooterDistance() {
        switch (shooterState) {
            case 0:
                robot.setShooterPower(0.47);
                shooterDistance = "Near";
                shooterState = 1;
                break;

            case 1:
                robot.setShooterPower(0.6);
                shooterDistance = "Far";
                shooterState = 0;
                break;
        }

        telemetry.addData("Shooter Distance", shooterDistance);
    }
}

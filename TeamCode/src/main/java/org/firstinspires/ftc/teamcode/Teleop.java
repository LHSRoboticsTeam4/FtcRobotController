package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Manually control robot", group = "")
public class Teleop extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware(this );
    private final RobotWheels robotwheels = new RobotWheels(this, robot);

    @Override
    public void runOpMode() {
        robot.init();
        robotwheels.init();

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            robot.startSpinTake();
            robot.startSideServos();
        }

        while (opModeIsActive()) {
            robotwheels.manuallyDriveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad1.x) {
                toggleShooterDistance();
                //Sleep to mitigate "long" presses of gamepad button.
                sleep(500);
            }

            if (gamepad1.y) {
                robot.setTopConveyPower(1);
            }
            else {
                robot.setTopConveyPower(0);
            }
        }
    }
    private int shooterState = 0;
    private String shooterDistance = null;
    private void toggleShooterDistance() {
        switch (shooterState) {
            case 0:
                robot.setShooterPower(1);
                shooterDistance = "Near";
                shooterState = 1;
                break;
            case 1:
                robot.setShooterPower(.9);
                shooterDistance = "Far";
                shooterState = 0;
                break;
        }

        telemetry.addData("Shooter Speed", shooterDistance);
        telemetry.update();
    }
}
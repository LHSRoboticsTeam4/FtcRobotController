package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Manually control robot", group = "")
public class Teleop  extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this );
        robot.init();
        RobotWheels robotwheels = new RobotWheels(this, robot);
        robotwheels.init();

        //Telemetry telemetry1 = FtcDashboard.getInstance().getTelemetry();

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            robotwheels.manuallyDriveRobot(gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x);

            if (gamepad1.dpad_up) {
                robot.moveSliderUp();
            }
            if (gamepad1.dpad_down) {
                robot.moveSliderDown();
            }
            if (gamepad1.right_bumper) {
                robot.intakeArmUp();
            }
            if (gamepad1.left_bumper) {
                robot.intakeArmDown();
            }
            if (gamepad1.a) {
                robot.openGrabber();
            }
            if (gamepad1.b) {
                robot.closeGrabber();
            }
            if (gamepad1.y){
                robot.outtake();
            }
            if (gamepad1.x){
                robot.intake();
            }
            if (!gamepad1.x && !gamepad1.y) {
                robot.stopIntake();
            }
            if (/*gamepad2.x*/gamepad2.right_trigger > 0.0) {
                robot.boxServoUp();
            }
            if (/*gamepad2.y*/gamepad2.left_trigger > 0.0) {
                robot.boxServoDown();
            }
        }
    }
}
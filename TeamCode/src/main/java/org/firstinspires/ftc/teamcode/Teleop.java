package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


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
                robot.moveSliderDown();
            }
            else if (gamepad1.dpad_down) {
                robot.moveSliderUp();
            }
            else robot.stopSlider();

            if(gamepad1.dpad_left) {
                robot.intakeArmMid();
            }
            if (gamepad1.right_bumper) {
                robot.intakeArmUp();
            }
            if (gamepad1.left_bumper) {
                robot.intakeArmDown();
            }
            if (gamepad2.right_bumper) {
                robot.boxServoClose();
            }
            if (gamepad2.left_bumper) {
                robot.boxServoOpen();
            }
            if (gamepad2.y){
                robot.intakeGrabberClose();
            }
            if (gamepad2.x){
                robot.intakeGrabberOpen();
            }
            if (gamepad2.b/*gamepad2.right_trigger > 0.0*/) {
                robot.boxServoUp();
            }
            if (gamepad2.a/*gamepad2.left_trigger > 0.0*/) {
                robot.boxServoDown();
            }
        }
    }
}
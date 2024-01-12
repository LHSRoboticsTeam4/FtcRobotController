package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This 2023-2024 OpMode shows a way to drive using encoders to park from a starting position.
 */
@Autonomous(name = "AutoBlueClose", group = "")
public class AutoBlueClose extends LinearOpMode {

    @Override
    public void runOpMode() {

        RobotHardware robot = new RobotHardware(this);
        robot.init();

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {

            // Set these variables to a initial distance, you will need to change this number
            int driveLeftInches = -30;
            int driveRightInches = -30;
            robot.autoDriveRobot(driveLeftInches, driveRightInches);
            robot.driveToSpike(SpikeColor.BLUE,100,-.1 );
        }
    }

}   // end class

